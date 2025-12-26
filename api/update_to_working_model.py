import os

env_path = ".env"
if os.path.exists(env_path):
    with open(env_path, "r") as f:
        lines = f.readlines()
    
    with open(env_path, "w") as f:
        for line in lines:
            if line.startswith("GEMINI_MODEL="):
                f.write("GEMINI_MODEL=gemini-flash-latest\n")
            else:
                f.write(line)
    print("Updated .env successfully")
else:
    print(".env not found")
