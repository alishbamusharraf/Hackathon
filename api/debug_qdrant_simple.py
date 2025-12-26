try:
    from qdrant_client import QdrantClient
    print("Import successful")
    client = QdrantClient(location=":memory:")
    print(f"Client: {client}")
    print(f"Has search: {hasattr(client, 'search')}")
    print(f"Methods: {[m for m in dir(client) if not m.startswith('_')]}")
except Exception as e:
    print(f"Error: {e}")
