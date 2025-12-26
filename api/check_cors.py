import os
from dotenv import load_dotenv

load_dotenv()
print(f"CORS_ORIGINS from env: {os.getenv('CORS_ORIGINS')}")
