import vineyard

def create_v6d_client(socket : str ="/var/run/vineyard.sock") -> vineyard.Client:
    v6d_client = vineyard.connect(socket)
    return v6d_client