# gimbal_kurulum — Fiziksel gimbalin repo DIŞI parçası

Gimbal iki parçadır; **bu klasör olmadan repo tek başına yetmez**:

| Parça | Nerede | Ne |
|---|---|---|
| 1 | Bu repo (gimbal dalı) | `models/suru_drone_*/model.sdf` (gimbal include + kamera tilt pivotunda), `bbox_to_redis --tilt` zinciri, `yildizlar_gimbal.eklem_acisi`, `tools/gz_gimbal.py`, testler. **Dünya dosyaları DEĞİŞMEDİ.** |
| 2 | `~/ardupilot_gazebo` (repo dışı) | `GimbalSmall2dPlugin` (Gazebo 11 portu + STABILIZE modu + hız-servo) ve `gimbal_small_2d` modeli (mesh'ler dahil — tilt collision'ı mesh kullandığı için gerekli). Bu klasör o parçanın kopyası + kurucusu. |

## Takım arkadaşı kurulumu (bizim setup varsayımıyla)

Önkoşullar (zaten kuruluysa atla): Ubuntu 20.04 + ROS Noetic + Gazebo 11
(classic) + ArduPilot SITL derlenmiş (`~/ardupilot`) + SwiftGust/khancyr
türevi `~/ardupilot_gazebo` (ArduPilotPlugin derlenen ağaç) + `redis-server`
+ `pymavlink/mavproxy` + python3 `numpy opencv rospy`.

```bash
git clone <repo> && cd <repo> && git checkout gimbal
./gimbal_kurulum/kur.sh          # plugin + model -> $ARDUPILOT_GAZEBO_DIR, derler
python3 tools/gimbal_headless_test.py   # SONUC: PASS beklenir (~2 dk)
python3 tools/gimbal_ucus_test.py       # SITL uçuş kanıtı: gövde ±35° iken kamera <0.65° (~4 dk)
```

Sonrası normal akış: `./yildizlar_gudum.sh --headless` ya da tek komut deneme
`SURE=300 tools/senaryo.sh`. Gimbal VARSAYILAN AÇIK; gövdeye-sabit eski
davranış için `--gimbalsiz` / `YILDIZ_GIMBAL=0`.

- Komut arayüzü: `/gazebo/default/iris-N/gimbal_tilt_cmd` (rad, DÜNYA
  elevasyonu, pozitif=yukarı), gerçek açı `.../gimbal_tilt_status` (~18 Hz).
- Tilt tek kaynaktan: `YILDIZ_TILT = atan(down/back)` (`scripts/standoff_geom.sh`;
  ayarı `tools/tilt_ayarla.py`).
- Ayrıntı + ölçülmüş tuzaklar (implicit_spring_damper çakışması, atalet/EKF,
  tembel render...): `NOTLAR_GIMBAL.md`.

## Farklı kurulumlar için notlar

- `ARDUPILOT_GAZEBO_DIR` farklıysa: `ARDUPILOT_GAZEBO_DIR=/yol ./gimbal_kurulum/kur.sh`
  ve aynı env ile `yildizlar_gudum.sh`.
- `kur.sh` CMakeLists'teki `GAZEBO_VERSION < 8.0` kapısını tanıyıp kaldırır;
  farklı bir CMake düzeninde uyarı basar (hedefi elle koşulsuz yapın).
- Kaynak model: SwiftGust/ardupilot_gazebo `models_gazebo/gimbal_small_2d`
  (bizim kopyada: limitler ±90°, kütle ~7 g, görseller kamera kadrajını
  işgal ettiği için kaldırıldı, ÇARPIŞMALAR AÇIK, plugin stabilize+servo
  parametreleri SDF'te).
