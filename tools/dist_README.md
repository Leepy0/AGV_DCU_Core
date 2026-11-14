# AGV_DCU_Core 배포 묶음

코어를 빌드한 lib 과 공개 헤더다. 소스·시험·문서는 없다(소스는 이 저장소의 `master` 와 버전 태그). 앱은 이 묶음을 앱 저장소의 `core/` 폴더에 그대로 둔다.

| 파일 | 내용 |
|---|---|
| `VERSION` | 코어 버전, 소스 커밋, lib 을 만든 컴파일러(cl2000) 버전 |
| `MANIFEST.sha256` | 묶음 파일마다 SHA-256. 글 파일은 줄 끝을 LF 로 맞춘 뒤, `.lib` 은 그대로 |
| `include/agv/*.h` | 공개 헤더 |
| `lib/core.lib` | 개발용(`AGV_DEV_TESTS` 1, 튜닝 t3·`TST`·조향 기록기 포함) |
| `lib/core_dev0.lib` | 현장용(`AGV_DEV_TESTS` 0) |
| `cmd/*.cmd` | 링커 명령 파일 |
| `tools/build_site.ps1` | 사이트 파일을 컴파일하고 lib 과 링크한다 |

## 받기

```
git clone --depth 1 --branch dist-vX.Y.Z https://github.com/Leepy0/AGV_DCU_Core.git core
```

`X.Y.Z` 는 받을 버전이다(예 `dist-v0.55.0`). 버전마다 태그 `dist-vX.Y.Z` 가 있고 브랜치 `dist` 는 마지막 버전이다. 받은 폴더의 `.git` 은 지워도 된다.

## 바꾸는 단위는 폴더 통째

`core/` 폴더를 통째로 지우고 새 묶음으로 바꾼다. **lib 파일만 바꾸면 안 된다.**

- lib 은 같은 버전의 헤더와 짝이다. 구조체 배치·표 칸 인코딩·함수 인자가 버전마다 바뀔 수 있고, 헤더와 lib 이 어긋나면 컴파일·링크가 되어도 엉뚱하게 돌 수 있다.
- 그래서 두 가지로 막는다. (1) `MANIFEST.sha256`: 앱이 빌드 전에 묶음 파일의 해시를 보고 하나라도 다르면 멈춘다. (2) `agv/version.h` 가 `site_register` 를 버전 붙은 이름(`site_register_v0_55_0` 꼴)으로 바꾼다. 다른 버전의 헤더로 컴파일한 사이트는 링크에서 그 심볼을 못 찾아 실패한다.

## 빌드

```
powershell -ExecutionPolicy Bypass -File core/tools/build_site.ps1 -Site <사이트 폴더> -Out <출력 경로(.out 없이)> [-ObjDir <폴더>] [-SiteDef NAME=1]
```

- 컴파일 옵션·링크 순서·링커 명령 파일은 lib 을 만든 것과 같다(`-v28 -ml -mt --c99 --float_support=softlib -O2 --opt_for_speed=2`, `cmd/F2808_app_direct.cmd`).
- 사이트의 `site.h` 에 `#define AGV_DEV_TESTS 0` 이 있으면 `lib/core_dev0.lib`, 없으면 `lib/core.lib` 를 링크한다.
- 쓰는 cl2000 의 버전이 `VERSION` 과 다르면 경고한다. TI C2000 CGT 18.12.2.LTS 를 쓴다.
