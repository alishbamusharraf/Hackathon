import google.generativeai as genai
import os
from dotenv import load_dotenv

load_dotenv()
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

model_name = "gemini-flash-latest"
print(f"Testing {model_name}...")
try:
    model = genai.GenerativeModel(model_name)
    response = model.generate_content("Hi")
    print(f"Success! Response: {response.text}")
except Exception as e:
    print(f"Error with {model_name}: {e}")
