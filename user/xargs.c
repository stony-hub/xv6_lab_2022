#include "kernel/types.h"
#include "kernel/stat.h"
#include "user/user.h"
#include "kernel/param.h"

int main(int argc, char *argv[]) {
  if (argc == 1) {
    fprintf(2, "usage: xargs [OPTION]... COMMAND [INITIAL-ARGS]...\n");
    exit(0);
  }
  
  char **nargv = (char**)malloc(sizeof(char*) * MAXARG), buffer[128], *p = buffer;
  int a;
  for (a = 0; a < argc - 1; a++) {
    nargv[a] = (char*)malloc(sizeof(char) * (strlen(argv[a+1]) + 1));
    strcpy(nargv[a], argv[a+1]);
  }

  for (; read(0, p, 1), *p;) {
    if (*p == '\n') {
      *p = 0;
      nargv[a] = (char*)malloc(sizeof(char) * (strlen(buffer) + 1));
      strcpy(nargv[a], buffer);
      a++;

      memset(buffer, 0, sizeof(buffer));
      p = buffer;
    }
    else p++;
  }

  nargv[a] = 0;
  
  if (fork() == 0) {
    if (exec(argv[1], nargv) == -1) {
      fprintf(2,"xargs: exec error\n");
      exit(1);
    }
  }

  wait(0);
  exit(0);
}
