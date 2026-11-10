/*
 * site.h - 앱이 구현하는 진입점
 *
 * site_register: 부팅 때 파라미터를 읽기 전에 한 번 부른다. param_default_type, io_bind,
 *   guide_configure, seq_bind, tagact_bind 를 여기서 호출한다. 프로파일은 파라미터를 읽은 뒤
 *   코어가 seq_pick_profile 로 다시 고른다.
 * site_manual: 10 ms 마다 부른다 (옛 DA_Manual_Act).
 */
#ifndef AGV_SITE_H
#define AGV_SITE_H
void site_register(void);
void site_manual(void);
#endif
