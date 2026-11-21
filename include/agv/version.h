/*
 * version.h - 코어 버전 이름(한 곳). tools/make_dist.ps1 이 배포 태그와 같은지 본다.
 *
 * 버전 맞춤 검사: site_register 를 버전 붙은 이름(예 site_register_v0_55_0)으로 바꾼다. 코어 lib 은 그 이름을 부르고
 * 사이트는 같은 헤더로 그 이름을 정의하므로, 다른 버전의 헤더로 컴파일한 사이트는 링크에서 그 심볼을 못 찾고 실패한다.
 * agv/types.h·agv/site.h 가 포함하므로 사이트 코드는 고치지 않는다.
 */
#ifndef AGV_VERSION_H
#define AGV_VERSION_H

#define AGV_CORE_VER           v0_61_0      /* 태그 이름의 점을 밑줄로. 릴리스마다 여기만 바꾼다 */

#define AGV_VER_CAT2(a, b)     a##_##b
#define AGV_VER_CAT(a, b)      AGV_VER_CAT2(a, b)
#define site_register          AGV_VER_CAT(site_register, AGV_CORE_VER)

#endif /* AGV_VERSION_H */
