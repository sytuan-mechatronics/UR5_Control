# Kich Ban Cau Hoi Phan Bien / Bao Ve

De tai: He thong UR5 pick-and-place dung camera Orbbec Femto Mega, YOLO11, hand-eye calibration va pneumatic gripper.

## 1. Mo Dau Bao Ve 1-2 Phut

Thua thay/co, de tai cua em xay dung he thong dieu khien robot UR5 de tu dong nhan dien, gap va dat phoi. He thong gom PC2 chay Flask server, robot UR5 CB3, camera Orbbec Femto Mega, model YOLO11 de phat hien phoi, hand-eye calibration de chuyen toa do tu anh sang he robot, va gripper khi nen de gap phoi.

Em chia qua trinh thuc nghiem thanh 3 giai doan:

- Phase 1: kiem tra motion co ban va gripper, chua dung camera.
- Phase 2: them camera, YOLO va bien doi toa do de gap 1 phoi.
- Phase 3: chay vong lap tu dong nhieu phoi tren khay, co rescan, retry va exclusion list.

Ket qua thuc nghiem ngay 26/06/2026:

- Phase 1: 8/8 lan thanh cong.
- Phase 2: 21/21 lan gap 1 phoi thanh cong, confidence trung binh khoang 0.932.
- Phase 3: 10 job, 50/50 lan pick thanh cong, khong can retry.

Diem chinh cua de tai la em khong chi dieu khien robot di theo diem co dinh, ma da tich hop day du chuoi xu ly: camera -> nhan dien -> tinh toa do 3D -> chuyen sang he robot -> dieu khien pick-place -> ghi log thuc nghiem.

## 2. Cau Hoi Tong Quan He Thong

### 1. De tai cua em giai quyet bai toan gi?

Tra loi:
De tai giai quyet bai toan tu dong gap phoi bang robot UR5 dua tren thi giac may. Thay vi day san tung diem gap co dinh, he thong dung camera de phat hien vi tri phoi, tinh toa do trong he robot va dieu khien robot gap phoi tu dong.

Neu bi hoi xoay:
Trong pham vi hien tai, bai toan duoc gioi han voi phoi dat tren khay va nam trong vung nhin camera. He thong phu hop cho cac bai toan cap phoi co vi tri lap lai, can tinh on dinh va kha nang ghi log thuc nghiem.

### 2. Kien truc he thong gom nhung thanh phan nao?

Tra loi:
He thong gom PC1 dieu phoi workflow, PC2 chay Flask server, UR5 CB3, camera Orbbec Femto Mega, YOLO11, module hand-eye calibration, va pneumatic gripper. PC1 gui lenh REST API sang PC2, PC2 thuc hien chu trinh robot va tra trang thai job.

Y chinh can nhan manh:
- PC2 la bo dieu khien robot va vision.
- REST API giup tach workflow tong voi dieu khien robot.
- Robot dung 3 kenh: Dashboard 29999, URScript 30002, RTDE 30004.

### 3. Vi sao chia thanh 3 phase?

Tra loi:
Em chia thanh 3 phase de kiem thu tang dan do phuc tap va giam rui ro khi chay robot that.

- Phase 1 kiem tra robot va gripper.
- Phase 2 kiem tra vision va pick 1 phoi.
- Phase 3 kiem tra luong tu dong nhieu phoi.

Neu bi hoi "tai sao khong test full ngay":
Robot that co rui ro va moi loi co the den tu nhieu nguon. Chia phase giup co lap loi: neu Phase 1 fail thi loi thuoc motion/gripper; neu Phase 2 fail thi tap trung vao vision/transform; neu Phase 3 fail thi tap trung vao loop logic, retry hoac quan ly trang thai.

### 4. He thong cua em khac gi voi robot chay diem co dinh?

Tra loi:
Robot chay diem co dinh chi di den toa do da day san. He thong cua em co them nhan dien bang camera va tinh toa do pick theo vi tri phoi thuc te. Vi vay khi phoi thay doi vi tri trong khay, robot van co the tinh target moi va gap.

## 3. Cau Hoi Ve Vision Va YOLO

### 5. Vi sao chon YOLO11?

Tra loi:
YOLO co toc do inference nhanh, phu hop ung dung real-time va de trien khai trong Python. Trong thuc nghiem, YOLO11 cho confidence on dinh, trung binh khoang 0.932 o Phase 2 va 0.932 o Phase 3, cho thay phat hien phoi tot trong dieu kien setup hien tai.

Neu bi hoi ve han che:
YOLO phu thuoc vao dataset va dieu kien anh sang. Neu doi loai phoi, anh sang, goc chup hoac background, can bo sung du lieu va fine-tune lai model.

### 6. Confidence cao co dong nghia voi gap chinh xac khong?

Tra loi:
Khong hoan toan. Confidence chi noi model tin rang do la phoi. Gap chinh xac con phu thuoc depth, camera intrinsics, hand-eye calibration, TCP, offset pick va co khi ca sai so co khi. Vi vay em danh gia ca ket qua pick, retry count va success rate, khong chi dua vao confidence.

### 7. Tu pixel anh sang toa do robot duoc tinh nhu the nao?

Tra loi:
Pipeline la:

1. YOLO tra ve vi tri phoi tren anh, lay pixel trung tam hoac diem pick.
2. Lay depth tai diem do de co Z trong camera.
3. Dung intrinsics camera de doi pixel `(u, v, depth)` sang diem 3D trong camera.
4. Dung hand-eye calibration `T_cam_to_tcp` va pose TCP tu RTDE de doi sang he base robot.
5. Cong them offset/correction neu can, sau do robot di den target pick.

Cong thuc y tuong:
`T_base_cam = T_base_tcp @ T_cam_to_tcp`

### 8. Neu depth sai thi he thong bi anh huong ra sao?

Tra loi:
Depth sai se lam sai toa do 3D, dac biet theo truc Z va co the lam gripper tiep can qua cao hoac qua thap. De giam rui ro, he thong co loc depth, dung scan pose on dinh, dung offset pick va kiem tra invalid depth. Trong thuc nghiem, depth dao dong 233-270 mm va van pick thanh cong 50/50.

### 9. Vi sao can hand-eye calibration?

Tra loi:
Camera va robot co 2 he toa do khac nhau. Hand-eye calibration cho biet quan he hinh hoc giua camera va TCP. Neu khong co ma tran nay, diem phoi trong anh khong the chuyen chinh xac thanh toa do robot de gap.

### 10. Sai so gap den tu dau?

Tra loi:
Cac nguon sai so chinh:
- Sai so detection cua YOLO.
- Nhieu/sai depth.
- Sai so intrinsics camera.
- Sai so hand-eye calibration.
- Sai so TCP/gripper.
- Do co khi, do phoi, ap suat khi nen.
- Sai so do robot khi di chuyen va moi truong anh sang.

## 4. Cau Hoi Ve Robot UR5 Va Dieu Khien

### 11. Vi sao dung URScript, Dashboard va RTDE?

Tra loi:
Moi kenh phuc vu mot muc dich:

- Dashboard dung cho trang thai robot, power, brake release, safety.
- URScript dung gui lenh chuyen dong `movej`, `movel`.
- RTDE dung doc trang thai thuc te cua robot, dac biet TCP pose khi can bien doi toa do.

### 12. Khi nao dung movej, khi nao dung movel?

Tra loi:
`movej` dung cho chuyen dong nhanh theo khop, vi du home den scan pose. `movel` dung khi can di theo duong thang trong khong gian Cartesian, dac biet luc tiep can va ha xuong diem pick de an toan va de kiem soat.

### 13. Vi sao toc do pick approach thap hon toc do linear chung?

Tra loi:
Toc do ha vao phoi thap hon de giam va cham, tranh lam lech phoi va bao ve gripper. Trong cau hinh thuc nghiem, toc do linear la 0.1 m/s, con toc do tiep can pick la 0.05 m/s.

### 14. He thong dam bao an toan nhu the nao?

Tra loi:
He thong chay theo tung phase, gioi han 1 job tai 1 thoi diem, co abort job, co move ve home khi loi, co mo gripper khi exception, va co nguoi giam sat E-stop khi chay robot that. Ngoai ra, cac pose nhu HOME, SCAN, PLACE duoc day truoc de nam trong workspace an toan.

Neu bi hoi "co an toan cong nghiep chua":
Em nen tra loi that:
He thong hien tai la muc prototype/thuc nghiem. De dat muc cong nghiep can bo sung risk assessment, safety PLC, vung an toan, interlock, enclosure va quy trinh E-stop chuan.

### 15. Tai sao chi cho phep 1 job chay tai 1 thoi diem?

Tra loi:
Vi robot la tai nguyen vat ly duy nhat. Neu 2 job cung dieu khien robot, co the gay xung dot lenh va nguy hiem. Do do JobStore va active job lock dam bao chi mot chu trinh duoc chay.

## 5. Cau Hoi Ve Gripper Khi Nen

### 16. Vi sao chon pneumatic gripper?

Tra loi:
Pneumatic gripper don gian, phan hoi nhanh, luc kep on dinh va phu hop voi bai toan gap phoi co hinh dang lap lai. Trong thuc nghiem, thoi gian dong trung binh 511.5 ms, mo 314.5 ms va khong co loi trong 50 picks.

### 17. Neu gap that bai thi xu ly the nao?

Tra loi:
Trong logic he thong co retry. Neu grip fail, robot rut ve approach pose, mo gripper, sau do thu lai toi so lan retry toi da. Neu van fail thi bao loi va dua he thong ve trang thai an toan. Trong ket qua hien tai chua co lan nao can retry.

### 18. Lam sao biet gripper da gap thanh cong?

Tra loi:
Trong setup hien tai, he thong dung phan hoi gripper/logic dieu khien va ket qua chu trinh de danh gia. Neu nang cap, co the them cam bien ap suat, cam bien hanh trinh hoac load feedback de xac nhan vat that su nam trong gripper.

## 6. Cau Hoi Ve Thuc Nghiem Va So Lieu

### 19. Ket qua thuc nghiem chinh la gi?

Tra loi:
Ket qua chinh:

- Phase 1: 8/8 thanh cong, motion va gripper on dinh.
- Phase 2: 21/21 pick 1 phoi thanh cong, confidence trung binh 0.932, retry 0.
- Phase 3: 10 job, moi job 5 phoi, tong 50/50 picks thanh cong, retry 0.

### 20. Ket qua 100% co dang tin khong?

Tra loi:
Ket qua 100% dang tin trong pham vi dieu kien thuc nghiem da dat ra: phoi tren khay, camera scan pose co dinh, anh sang on dinh, loai phoi co dinh. Tuy nhien khong nen khang dinh tong quat cho moi moi truong. De tang do tin cay thong ke, can tang so lan lap, thay doi dieu kien anh sang, vi tri phoi, va test them cac truong hop loi.

Day la cau tra loi nen dung neu thay hoi xoay.

### 21. Vi sao Phase 3 job dau tien lau hon?

Tra loi:
Job dau tien co the gom overhead khoi tao: camera stream LAN, flush frame buffer, ket noi va settle ban dau. Sau do he thong on dinh hon nen thoi gian job giam. Trong bao cao, em co tach nhan xet warm-up de khong lay job dau lam dai dien duy nhat.

### 22. Tai sao Phase 2 mat khoang 82s, Phase 3 230s cho 5 phoi?

Tra loi:
Phase 2 gom motion, scan, inference, tinh toa do, pick-place 1 phoi. Phase 3 lap lai cho nhieu phoi nen tong thoi gian tang. Tuy nhien tinh theo moi phoi o trang thai on dinh thi Phase 3 co the dat khoang 43-44s/phoi khi tinh ca job, va part duration chi tiet khoang 6-9s cho tung phan xu ly phoi, tuy cach dinh nghia moc do thoi gian.

Can noi ro:
`duration_s` cua job tinh toan bo chu trinh, con `part_duration_s` la thoi gian rieng cho phoi trong log chi tiet, nen hai so nay khong dong nhat.

### 23. Tai sao so log Phase 3 chi tiet la 4 job, nhung job_summary co 10 job?

Tra loi:
`job_summary.csv` ghi tong ket cho 10 job Phase 3. `scenario3_phase3.csv` hien co log chi tiet theo tung phoi cho 4 job. Vi vay khi trinh bay can noi ro nguon du lieu: tong ket success rate dua tren 10 job, con thong ke part_duration chi tiet dua tren 4 job co log tung phoi.

### 24. Mau thuc nghiem co du lon khong?

Tra loi:
Voi muc tieu prototype va kiem chung pipeline, so mau 8/21/10 job la du de chung minh he thong chay duoc va on dinh trong dieu kien phong lab. Tuy nhien de ket luan do tin cay cong nghiep, can mo rong so mau len hang tram/ nghin chu ky, them test nhieu dieu kien anh sang, sai lech vi tri va phoi loi.

### 25. Neu thay hoi "tai sao khong co sai so dinh vi mm trong Phase 2?"

Tra loi:
Trong file log co cot `localization_error_mm` nhung chua duoc dien tu phep do doc lap. Ket qua hien tai danh gia theo ket qua pick thanh cong va repeatability. De hoan thien hon, can do toa do phoi thuc te bang jig/reference hoac teach point, sau do so sanh voi toa do vision de tinh sai so mm.

## 7. Cau Hoi Ve Thuat Toan Phase 3

### 26. Phase 3 tranh gap lai cung mot phoi nhu the nao?

Tra loi:
He thong co rescan sau moi lan pick va exclusion list cac vi tri da gap. Sau khi pick thanh cong, vi tri do duoc dua vao danh sach loai tru, lan scan tiep theo se chon phoi con lai.

### 27. Neu YOLO phat hien nham vi tri trong khay thi sao?

Tra loi:
Co the gay target sai. De giam rui ro, he thong dung correction map theo slot, depth check va vung khay da biet truoc. Huong cai tien la them segmentation, tracking sau moi pick, va validate bang tray slot geometry.

### 28. Neu co 0 phoi tren khay thi he thong lam gi?

Tra loi:
Neu initial scan khong thay phoi, he thong ket thuc voi trang thai no parts found/done tuy theo logic job, khong thuc hien pick khong. Day la mot kich ban can test rieng trong Phase 3/5 de chung minh he thong khong gap khong va khong treo job.

## 8. Cau Hoi Ve API Va Phan Mem

### 29. API chinh gom nhung endpoint nao?

Tra loi:
API chinh gom:

- `/execute`: tao job moi.
- `/status/<job_id>`: xem trang thai job.
- `/abort/<job_id>`: dung job dang chay.
- `/health`: kiem tra suc khoe he thong.

### 30. Vi sao dung Flask?

Tra loi:
Flask gon nhe, de trien khai REST API noi bo giua PC1 va PC2. Voi muc tieu prototype/thuc nghiem, Flask du dap ung va de debug. Neu len san xuat co the bo sung WSGI server, auth, logging tap trung va monitoring.

### 31. JobStore luu trong memory co han che gi?

Tra loi:
Neu server restart thi mat trang thai job trong memory. Hien tai du cho thuc nghiem vi log CSV van duoc ghi ra file. De san xuat, nen chuyen sang SQLite/PostgreSQL hoac message queue de luu trang thai ben vung.

### 32. Neu PC1 mat ket noi voi PC2 khi robot dang chay?

Tra loi:
Job tren PC2 van co the tiep tuc vi da spawn thread noi bo. PC1 co the goi lai `/status` khi ket noi lai. Tuy nhien de an toan hon, ban san xuat nen co heartbeat/watchdog, neu mat ket noi qua nguong thi robot pause hoac abort.

## 9. Cau Hoi Ve Gioi Han Va Huong Phat Trien

### 33. Gioi han lon nhat cua de tai la gi?

Tra loi:
Gioi han lon nhat la thuc nghiem moi trong dieu kien lab co kiem soat: mot loai phoi, khay co dinh, anh sang tuong doi on dinh, chua test rong rai voi phoi loi, occlusion manh, thay doi anh sang lon hoac moi truong cong nghiep.

### 34. Neu doi sang loai phoi khac thi can lam gi?

Tra loi:
Can thu thap dataset moi, fine-tune YOLO, kiem tra lai diem gap, luc gap, offset pick va co the thay doi gripper. Neu kich thuoc/hinh dang phoi thay doi nhieu thi can calibration lai correction map.

### 35. Neu camera bi lech sau khi lap dat thi sao?

Tra loi:
Neu camera lech, hand-eye calibration khong con dung, toa do pick se sai. Can co quy trinh kiem tra calibration dinh ky va recalibrate khi thao lap camera/TCP.

### 36. Huong phat trien tiep theo la gi?

Tra loi:
Huong phat trien:

- Tang so mau thuc nghiem va test dieu kien moi truong khac nhau.
- Bo sung do sai so dinh vi bang mm.
- Them cam bien xac nhan grip.
- Toi uu thoi gian chu trinh.
- Tich hop day du voi MIR va workflow tong.
- Luu job vao database thay vi memory.
- Bo sung safety cong nghiep: interlock, safety zone, watchdog.

## 10. Cau Hoi Kho / Phan Bien Manh

### 37. Ket qua 100% co phai do bai toan qua de?

Tra loi de giu diem:
Dung la bai toan da duoc gioi han de phu hop voi muc tieu thuc nghiem: phoi va khay co dinh, camera co scan pose on dinh. Tuy nhien day la buoc can thiet de chung minh pipeline robot-vision hoat dong dung. Sau khi pipeline on dinh, em moi co co so de mo rong sang dieu kien kho hon nhu anh sang thay doi, phoi dat lech nhieu, occlusion, hoac nhieu loai phoi.

### 38. Neu thầy nói "em chỉ dùng YOLO có sẵn, đóng góp ở đâu?"

Tra loi:
Dong gop cua em nam o viec tich hop YOLO vao he robot that: lay depth, bien doi toa do camera sang robot, dieu khien UR5 theo target dong, quan ly job, retry, gripper, logging va thuc nghiem. YOLO chi la mot module trong pipeline; phan quan trong la lam cho detection tro thanh lenh robot an toan va lap lai duoc.

### 39. Neu thầy hỏi "co chung minh duoc hand-eye calibration dung khong?"

Tra loi:
Em chung minh gian tiep bang ket qua pick thanh cong tren nhieu slot va nhieu lan lap. Tuy nhien de chung minh dinh luong day du, can them bang sai so mm giua toa do vision du doan va toa do reference do bang teach point/jig. Day la muc em de xuat bo sung trong huong phat trien.

### 40. Neu thầy hỏi "neu robot gap sai thi ai chiu trach nhiem?"

Tra loi:
Trong muc thuc nghiem, robot chay duoi su giam sat cua nguoi van hanh va E-stop. Ve ky thuat, he thong co abort, exception handling va dua robot ve home. Tuy nhien de dua vao san xuat can co tang safety doc lap nhu safety PLC, interlock va risk assessment theo chuan.

### 41. Neu thầy hỏi "tai sao khong dung PLC thay Flask?"

Tra loi:
PLC phu hop dieu khien cong nghiep va safety. Flask phu hop cho phan xu ly vision, AI va REST integration trong prototype. Trong he thong san xuat, co the ket hop: PLC xu ly safety va I/O thoi gian thuc, PC xu ly vision/AI va gui target cho robot.

### 42. Neu thầy hỏi "tai sao khong dung camera co dinh ma dung eye-in-hand?"

Tra loi:
Eye-in-hand giup camera di cung robot, linh hoat hon khi can quan sat nhieu vi tri va giam yeu cau lap dat camera ngoai. Tuy nhien no phu thuoc chat luong hand-eye calibration va scan pose. Camera co dinh co the don gian hon neu workspace khong thay doi.

### 43. Neu thầy hỏi "do confidence 0.93 co y nghia gi?"

Tra loi:
Confidence 0.93 cho thay model nhan dien phoi rat chac trong dieu kien anh hien tai. Nhung confidence khong phai xac suat thanh cong cua pick. Vi vay em ket hop confidence voi ket qua pick thuc te, retry count va success rate de danh gia toan he thong.

### 44. Neu thầy hỏi "he thong co real-time khong?"

Tra loi:
He thong khong phai real-time cung theo nghia dieu khien servo. Dieu khien servo van do controller UR dam nhan. PC2 thuc hien real-time o muc ung dung: nhan anh, suy luan, tinh target va gui lenh motion. Voi bai toan pick-place theo chu ky, muc nay la phu hop.

### 45. Neu thầy hỏi "neu anh sang thay doi thi sao?"

Tra loi:
Anh sang thay doi co the lam confidence giam hoac detection sai. De cai thien, co the co den chieu sang co dinh, bo sung anh training voi dieu kien anh sang khac, dung augmentation va dat nguong confidence phu hop.

## 11. Cau Hoi Theo Tung File / Module

### 46. `app.py` lam gi?

Tra loi:
`app.py` khoi tao Flask server, logging, shared robot/camera/gripper clients, dang ky API blueprint va chay server PC2.

### 47. `core/pick_place.py` lam gi?

Tra loi:
Day la state machine chinh cua chu trinh pick-place: ve home, den scan pose, chup anh, detect, tinh target, pick, place, retry, update job status va xu ly loi.

### 48. `vision/detector.py` lam gi?

Tra loi:
Module nay goi YOLO model de phat hien phoi trong anh va tra ve bounding box, confidence, target pixel.

### 49. `vision/calibration.py` lam gi?

Tra loi:
Module nay xu ly bien doi toa do: pixel + depth sang diem 3D camera, sau do tu camera frame sang base frame robot bang hand-eye matrix va TCP pose.

### 50. `results/*.csv` dung de lam gi?

Tra loi:
Day la log thuc nghiem. Moi file ung voi mot nhom ket qua: Phase 1, Phase 2, Phase 3, job summary va pick detections. Cac file nay duoc dung de tinh success rate, time trung binh, confidence va retry count trong bao cao.

## 12. Cau Tra Loi Ngan Can Hoc Thuoc

- Neu hoi muc tieu: "Tu dong gap phoi bang UR5 dua tren vision, khong phu thuoc hoan toan vao diem gap co dinh."
- Neu hoi pipeline: "Camera -> YOLO -> depth -> pixel to 3D -> hand-eye transform -> robot pick-place."
- Neu hoi ket qua: "Phase 1: 8/8, Phase 2: 21/21, Phase 3: 50/50 picks, retry 0."
- Neu hoi han che: "Moi test trong dieu kien lab co kiem soat, can tang mau va test dieu kien kho hon."
- Neu hoi dong gop: "Tich hop AI vision voi robot that, calibration, dieu khien UR5, gripper, retry, logging va thuc nghiem."
- Neu hoi an toan: "Prototype co abort, home recovery, single job; san xuat can safety PLC/interlock/risk assessment."
- Neu hoi cai tien: "Do sai so mm, them cam bien grip, tang dataset, toi uu cycle time, tich hop MIR va database."

## 13. Cau Hoi Nen Chu Dong Noi Truoc

Neu co co hoi, em nen chu dong noi 3 diem nay de tranh bi bat be:

1. "Ket qua 100% chi ket luan trong pham vi dieu kien thuc nghiem da xac dinh, khong khang dinh tong quat cho moi dieu kien cong nghiep."
2. "Cot sai so dinh vi mm hien chua co day du phep do doc lap, nen em danh gia chinh qua success rate, retry va repeatability; day la huong bo sung."
3. "Job dau Phase 3 co warm-up nen em tach nhan xet ve thoi gian on dinh sau warm-up."

