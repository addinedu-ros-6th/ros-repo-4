from multiprocessing import Pipe, Process
from control_server import ControlServer
from ai_server import AiServer 

def main():
    ai_pipe, control_pipe = Pipe(duplex=True)
    
    c_server = ControlServer()
    a_server = AiServer()

    ai_process = Process(target=c_server.run, args=(ai_pipe, ))
    control_process = Process(target=a_server.run, args=(control_pipe, ))

    ai_process.start()
    control_process.start()

    ai_process.join()
    control_process.join()

if __name__ == "__main__":
    main()
