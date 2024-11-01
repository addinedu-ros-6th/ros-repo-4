import subprocess

def main():
    # 각 스크립트를 서브프로세스로 실행
    try:
        subprocess.Popen(['python3', 'AI_server.py'])  # 첫 번째 스크립트 실행
        subprocess.Popen(['python3', 'tcp_ip_server.py'])  # 두 번째 스크립트 실행


        print("Both scripts are running...")
    except Exception as e:
        print(f"An error occurred while starting the scripts: {e}")

if __name__ == "__main__":
    main()
