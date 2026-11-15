/* DSL 통일 시험 공용: 바이트 비교. 호스트는 memcmp, cl2000 은 크기만(바이트는 asm 으로 따로 비교, tests/dsl/asmcmp.ps1) */
#ifdef AGV_HOST_BUILD
#include <stdio.h>
#include <string.h>
static int bad_;
#define SAME(a, b) do { if (sizeof(a) != sizeof(b) || memcmp(a, b, sizeof(a)) != 0) { bad_++; printf("DIFF %s vs %s\n", #a, #b); } \
                        else printf("same %s %s (%u bytes)\n", #a, #b, (unsigned)sizeof(a)); } while (0)
#define MAIN_BEGIN int main(void) {
#define MAIN_END   return bad_; }
#else
#define KP2_(l) keep_##l
#define KP_(l) KP2_(l)
#define SAME(a, b) const void *const KP_(__LINE__)[2] = { a, b }
#define MAIN_BEGIN
#define MAIN_END
#endif
#define SEN_1 IN_EXT_1
#define SEN_2 IN_EXT_2
#define SEN_3 IN_EXT_3
#define SEN_4 IN_EXT_4
#define SEN_5 IN_EXT_5
#define SEN_H IN_ACTIVE_HIGH(IN_EXT_6)
