#include <sys/param.h>
#include <sys/lock.h>
#include <sys/mutex.h>

void mtx_lock_spin(struct mtx *m) { }
int mtx_trylock_spin(struct mtx *m) { return (1); }
void mtx_unlock_spin(struct mtx *m) { }
