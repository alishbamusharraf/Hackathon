import os

env_path = ".env"
if os.path.exists(env_path):
    with open(env_path, "r") as f:
        lines = f.readlines()
    
    with open(env_path, "w") as f:
        for line in lines:
            if line.startswith("CORS_ORIGINS="):
                f.write("CORS_ORIGINS=http://localhost:3000,http://localhost:3001,http://127.0.0.1:3000,http://127.0.0.1:3001\n")
            else:
                f.write(line)
    print("Updated CORS in .env successfully")
else:
    print(".env not found")
