import opengen as og


def connect_optimizer(optimizer_name, ports_to_try):
    """
    Try to connect to the optimizer TCP server on available ports.
    Args:
        optimizer_name (str): Name of the optimizer.
        ports_to_try (list): List of ports to attempt connection.
    Returns:
        opengen.tcp.OptimizerTcpManager: Connected optimizer manager.
    """
    mng = None
    for port in ports_to_try:
        try:
            print(f"Trying to connect to TCP server on port {port}...")
            mng = og.tcp.OptimizerTcpManager(optimizer_name, port=port)
            mng.start()
            print(f"Successfully connected to TCP server on port {port}")
            return mng
        except Exception as e:
            print(f"Failed to connect on port {port}: {e}")
            if mng:
                try:
                    mng.kill()
                except:
                    pass
            mng = None
    print("Failed to connect to TCP server on any port.")
    print("Please make sure the optimizer was built correctly by running generator.")
    exit(1)