from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

url = os.getenv("QDRANT_URL")
api_key = os.getenv("QDRANT_API_KEY")

client = QdrantClient(url=url, api_key=api_key)
print(f"Client type: {type(client)}")
print(f"Has 'search': {hasattr(client, 'search')}")
print(f"Dir: {dir(client)}")
