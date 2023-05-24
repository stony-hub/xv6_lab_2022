#include "kernel/types.h"
#include "kernel/stat.h"
#include "user/user.h"

void func(int *pfd) {
  int a, b, new_pfd[2];

  close(pfd[1]);
  if (read(pfd[0], &a, sizeof(int)) == 0) {
    close(pfd[0]);
    exit(0);
  }
  printf("prime %d\n", a);

  pipe(new_pfd);
  if (fork() == 0) {
    close(pfd[0]);
    func(new_pfd);
  }
  close(new_pfd[0]);

  while (read(pfd[0], &b, sizeof(int))) {
    if (b % a != 0) {
        write(new_pfd[1], &b, sizeof(int));
    }
  }

  close(pfd[0]);
  close(new_pfd[1]);

  wait(0);
  exit(0);
}

int main(int argc, char* argv[]) {
  int pfd[2];
  pipe(pfd);

  if (fork() == 0) {
    func(pfd);
  }

  close(pfd[0]);
  for (int i=2; i<=35; i++) {
    write(pfd[1], &i, sizeof(int));
  }
  close(pfd[1]);

  wait(0);
  exit(0);
}
