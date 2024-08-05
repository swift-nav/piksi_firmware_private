#include <libpal/impl/thread/thread.h>
#include <libpal/thread/thread.h>
#include <pal_private.h>

#include "not_implemented.h"

/**
 * Default thread start routine
 * @param entry Unused
 * @param ctx Unused
 * @param stacksize Unused
 * @param prio Unused
 * @param thread Unused
 * @return PAL_INVALID
 */
static enum pal_error default_thread_create(pal_thread_t *thread,
                                            pal_thread_entry_t entry, void *ctx,
                                            size_t stacksize, uint8_t prio) {
  NOT_IMPLEMENTED();
  (void)thread;
  (void)entry;
  (void)ctx;
  (void)stacksize;
  (void)prio;
  return PAL_INVALID;
}

/**
 * Default set thread name routine
 * @param name Unused
 * @return PAL_INVALID
 */
static enum pal_error default_thread_set_name(const char *name) {
  NOT_IMPLEMENTED();
  (void)name;
  return PAL_INVALID;
};

/**
 * Default join thread routine
 * @param thread Unused
 * @param retval Unused
 * @return PAL_INVALID
 */
static enum pal_error default_thread_join(pal_thread_t thread, void **retval) {
  NOT_IMPLEMENTED();
  (void)thread;
  (void)retval;
  return PAL_INVALID;
}

/**
 * Default thread exit routine
 * @param code Unused
 */
static void default_thread_exit(void *code) {
  NOT_IMPLEMENTED();
  (void)code;
}

static enum pal_error default_thread_interrupt(pal_thread_t thread) {
  NOT_IMPLEMENTED();
  (void)thread;
  return PAL_INVALID;
}

static struct pal_impl_thread thread_impl = {
    .create = default_thread_create,
    .set_name = default_thread_set_name,
    .join = default_thread_join,
    .exit = default_thread_exit,
    .interrupt = default_thread_interrupt,
};

void pal_reset_impl_thread(void) {
  thread_impl.create = default_thread_create;
  thread_impl.set_name = default_thread_set_name;
  thread_impl.join = default_thread_join;
  thread_impl.exit = default_thread_exit;
  thread_impl.interrupt = default_thread_interrupt;
}

void pal_set_impl_thread(struct pal_impl_thread *impl) {
  assert(NULL != impl);
  assert(NULL != impl->create);
  assert(NULL != impl->set_name);
  assert(NULL != impl->join);
  assert(NULL != impl->exit);
  assert(NULL != impl->interrupt);

  thread_impl = *impl;
}

bool pal_has_impl_thread() {
  return thread_impl.create != default_thread_create;
}

enum pal_error pal_thread_create(pal_thread_t *thread, pal_thread_entry_t entry,
                                 void *ctx, size_t stacksize, uint8_t prio) {
  return thread_impl.create(thread, entry, ctx, stacksize, prio);
}

enum pal_error pal_thread_set_name(const char *name) {
  return thread_impl.set_name(name);
}
enum pal_error pal_thread_join(pal_thread_t thread, void **retval) {
  return thread_impl.join(thread, retval);
}
void pal_thread_exit(void *code) { return thread_impl.exit(code); }

enum pal_error pal_thread_interrupt(pal_thread_t thread) {
  return thread_impl.interrupt(thread);
}
