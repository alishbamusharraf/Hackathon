import asyncio
from qdrant_client import AsyncQdrantClient

async def main():
    client = AsyncQdrantClient(location=":memory:")
    print(f"Async Client: {client}")
    print(f"Methods: {[m for m in dir(client) if not m.startswith('_')]}")

if __name__ == "__main__":
    asyncio.run(main())
