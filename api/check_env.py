import os
from dotenv import load_dotenv

load_dotenv()
print(f"GEMINI_MODEL from env: {os.getenv('GEMINI_MODEL')}")
