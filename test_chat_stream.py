
import requests
import json

url = "http://localhost:8000/api/chat/stream"
data = {
    "message": "Hello, what is this book about?",
    "session_id": "test_session"
}

try:
    response = requests.post(url, json=data, stream=True)
    print(f"Status Code: {response.status_code}")
    for line in response.iter_lines():
        if line:
            print(line.decode('utf-8'))
except Exception as e:
    print(f"Error: {e}")
