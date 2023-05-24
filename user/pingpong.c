#include "kernel/types.h"
#include "kernel/stat.h"
#include "user/user.h"

int main(int argc, char* argv[]) {
  int pfd1[2], pfd2[2], pid;
  char msg[65];
  pipe(pfd1);
  pipe(pfd2);

  if (fork() != 0) { // parent
    pid = getpid();

    close(pfd1[0]);
    close(pfd2[1]);

    char *ping = "a";
    write(pfd1[1], ping, 1);

    if (read(pfd2[0], msg, 1) != 0)
      printf("%d: received pong\n", pid);

  } else { // child
    pid = getpid();

    close(pfd1[1]);
    close(pfd2[0]);

    if(read(pfd1[0], msg, 1) != 0)
      printf("%d: received ping\n", pid);

    write(pfd2[1], msg, 1);
  }

  exit(0);
}
