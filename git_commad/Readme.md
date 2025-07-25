

## git commad


### 처음에는 항상 상대방의 수정사항을 먼저 받아줘야합니다.

```
git pull
```

### 내가 수정한 것을 깃 원격 즉 웹상에 올리고 싶을때

```
git add .
git commit -m "update"
git push
```



<br>

## 시놀로지와도 git하고 연결 가능하다고 함!! 찾아보자

<br>
<br>

아래는 지금까지 실행하신 명령어를 목적별로 정리한 가이드입니다. 필요에 따라 해당 순서대로 따라 하시면 됩니다.

---

## 1. 원격 저장소 확인

```bash
git remote -v
```

* origin URL을 확인합니다.

## 2. 브랜치 현황 보기

```bash
git branch           # 로컬 브랜치
git branch -r        # 원격 브랜치
git branch -a        # 전체 브랜치
```

## 3. 새 브랜치 생성 및 체크아웃

* **로컬에만** 새 브랜치 생성

  ```bash
  git checkout -b jin
  ```
* **원격(origin)에만** 있는 브랜치 가져오기

  ```bash
  git checkout -b jin origin/jin
  ```
* Git 2.23 이상에서는 `switch` 사용 가능

  ```bash
  git switch -c jin             # 로컬 새 브랜치 생성 후 이동
  git switch jin                # 기존 브랜치로 이동
  git switch -c jin origin/jin  # 원격 브랜치에서 생성
  ```

## 4. 작업 커밋 & 원격에 브랜치 푸시

```bash
git add .
git commit -m "make jin branch"
git push --set-upstream origin jin
```

* `--set-upstream` (`-u`) 옵션으로 로컬 jin ↔ origin/jin 연결 설정

## 5. main 브랜치 최신화 → jin 브랜치에 병합

```bash
# 먼저 main 브랜치로 이동
git checkout main

# 원격 변경사항 가져오기
git pull

# jin 브랜치로 돌아가서 main 머지
git checkout jin
git merge main
```

* Fast‑forward만 발생했다면 자동으로 병합됩니다.

## 6. jin 브랜치 변경사항을 다시 main에 병합

```bash
git checkout main
git merge jin
```

## 7. 최종 결과 원격에 푸시

```bash
git push
```

---

위 순서대로 하면

1. 원격·로컬 브랜치 확인
2. jin 브랜치 생성/체크아웃
3. 커밋 후 origin에 푸시
4. main 최신화 → jin에 병합
5. jin 내용을 main에 병합
6. main 푸시


-------------------------

## 병합 충돌 해결방법 


- 2개의 내용을 가지고 합쳐서 진행할 수 있음
- 자동으로 해주는 것은 rebase 로 진행가능함
- 100메가가 넘어가는 파일을 올리면 막히게 된다. -> git을 삭제한다. soft reset 진행해서
- 분할 압축을 진행하여서 할 수도 잇음  
