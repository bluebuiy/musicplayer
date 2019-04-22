
#include <string>
#include <string.h>
#include <fstream>

#include <sys/types.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>


int main(void)
{
  std::ifstream file("../playlist.playlist", std::ios_base::in);
  if (!file)
  {
    printf("Failed to open file.\n");
    return 1;
  }
  
  
  std::string line;
  while (std::getline(file, line, '\n'))
  {
    line.pop_back();
    int pid = fork();
    if (pid == -1)
    {
      printf("Failed to fork\n");
      return 1;
    }
    if (pid == 0)
    {
      //freopen("playerlog.log", "a+", stdout); 
      char *args[3];
      char args_v[2][1024];
      
      strncpy(args_v[0], "./player.exe", 1023);
      args_v[0][1023] = 0;
      
      strncpy(args_v[1], line.c_str(), 1023);
      args_v[1][1023] = 0;
      
      args[0] = args_v[0];
      args[1] = args_v[1];
      args[2] = nullptr;
      
      printf("Invoking player...\n");
      
      execv("./player.exe", args);
      return -1;
    }
    else
    {
      int ret = 0;
      int err = EINTR;
      do
      {
        err = waitpid(pid, &ret, 0); 
      } while (errno == EINTR);
      if (err == -1)
      {
        printf("Failed to wait on child process. Terminating...\n");
        kill(pid, SIGKILL);
      }
    }
  }
  
  
  
  
  
  
}


