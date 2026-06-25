# optimizer/optimizer.py
import casadi as cs
import opengen as og
from model.dynamics import unicycle_dynamics
from optimizer.cost import stage_cost, terminal_cost
import os
import yaml


class MPCConfig:
    """Container for all NMPC parameters (loaded from the YAML config).

    Weights are diagonal: Q (stage state error), R (stage input effort) and
    QN (terminal state error). ``trajectory`` selects the reference pattern.
    """

    def __init__(self, state_dim=3, input_dim=2, sampling_time=0.1, horizon_len=20,
                 umin=None, umax=None, Q=None, R=None, QN=None, trajectory="figure8"):
        self.state_dim     = state_dim
        self.input_dim     = input_dim
        self.sampling_time = sampling_time
        self.horizon_len   = horizon_len
        self.umin          = umin if umin is not None else [-10, -10]
        self.umax          = umax if umax is not None else [10, 10]
        self.Q             = Q if Q is not None else [1, 1, 0.1]
        self.R             = R if R is not None else [0.01, 0.01]
        self.QN            = QN if QN is not None else [1, 1, 0.1]
        self.trajectory    = trajectory


    @classmethod
    def from_yaml(cls, yaml_path):
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        # Accept the legacy key "Qt" as a fallback for backward compatibility.
        QN = data.get('QN', data.get('Qt', [1, 1, 0.1]))
        return cls(
            state_dim     = data.get('state_dim', 3),
            input_dim     = data.get('input_dim', 2),
            sampling_time = data.get('sampling_time', 0.1),
            horizon_len   = data.get('horizon_len', 20),
            umin          = data.get('umin', [-10, -10]),
            umax          = data.get('umax', [10, 10]),
            Q             = data.get('Q', [1, 1, 0.1]),
            R             = data.get('R', [0.01, 0.01]),
            QN            = QN,
            trajectory    = data.get('trajectory', 'figure8'),
        )


def build_optimizer(config_path=None):
    cfg = MPCConfig.from_yaml(config_path) if config_path else MPCConfig()

    nx, nu, N = cfg.state_dim, cfg.input_dim, cfg.horizon_len
    dt = cfg.sampling_time

    # -----------------------------
    # Decision variables: U = [u_0; ...; u_{N-1}]
    # -----------------------------
    U = cs.MX.sym("U", nu*N)

    # -----------------------------
    # Parameters p =
    # [ x0 (nx) ;
    #   Xref_flat (nx*(N+1)) ;
    #   Uref_flat (nu*N) ;
    #   Q(nx) ; QN(nx) ; R(nu) ]
    # The runtime parameter vector built in main_NMPC.py must follow this order.
    # -----------------------------
    x0      = cs.MX.sym("x0", nx)
    Xref    = cs.MX.sym("Xref", nx*(N+1))
    Uref    = cs.MX.sym("Uref", nu*N)
    Qp      = cs.MX.sym("Q", nx)
    QNp     = cs.MX.sym("QN", nx)
    Rp      = cs.MX.sym("R", nu)
    P       = cs.vertcat(x0, Xref, Uref, Qp, QNp, Rp)


    # set the refrence vectors
    def xref_k(k):
        s = nx*k
        return Xref[s:s+nx]


    def uref_k(k):
        s = nu*k
        return Uref[s:s+nu]

    # Rollout + cost
    xk = x0
    total_cost = 0
    for k in range(N):
        uk = U[nu*k:nu*(k+1)]
        total_cost += stage_cost(xk, xref_k(k), Qp, uk, Rp)
        xk = unicycle_dynamics(xk, uk, dt)
    # terminal cost
    total_cost += terminal_cost(xk, xref_k(N), QNp)

    # Bounds on U: OpEn's Rectangle must match the full decision-variable
    # dimension (nu*N), so repeat the per-stage input bounds for every stage.
    umin = list(cfg.umin) * N
    umax = list(cfg.umax) * N
    bounds = og.constraints.Rectangle(umin, umax)

    # Build problem
    problem = og.builder.Problem(U, P, total_cost).with_constraints(bounds)

    build_dir = f"build/unicycle/horizon_{N}"
    sampling_time_str = str(cfg.sampling_time).replace('.', '_')
    opt_name = f"sampling_{sampling_time_str}"

    build_config = og.config.BuildConfiguration() \
        .with_build_directory(build_dir) \
        .with_tcp_interface_config()

    meta = og.config.OptimizerMeta().with_optimizer_name(opt_name)

    max_time = int(cfg.sampling_time * 1e6)
    solver_config = og.config.SolverConfiguration() \
        .with_tolerance(1e-6) \
        .with_initial_tolerance(1e-6) \
        .with_max_duration_micros(max_time) \
        .with_preconditioning(True)

    builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_config, solver_config)
    builder.build()

    # Verify
    optimizer_path = os.path.join(build_dir, opt_name)
    yaml_file = os.path.join(optimizer_path, "optimizer.yml")

    if os.path.exists(yaml_file):
        print(f"✓ Build verification: {yaml_file} exists")
    else:
        print(f"✗ Build verification failed: {yaml_file} not found")
        print(f"Expected path: {os.path.abspath(yaml_file)}")

    print("You can now run the optimized main_NMPC.py for faster simulation.")
    print("MPCConfig loaded from:", config_path if config_path else "default (class values)")
