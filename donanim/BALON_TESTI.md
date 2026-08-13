# Balon testi — donanım zinciri

Amaç: hedef telemetrisinden yalnız `range_m = ||hedef_ned-avci_ned||` kullanılır; LOS yönü, elevasyon ve kerteriz yalnız görüntüden gelir.

## Veri yolu

`Microhard hedef GLOBAL_POSITION_INT → HamTelemetriMenzil → tek skaler range_m`

`kamera/model → tracker_bbox → bbox_to_redis → tracker_bbox_stab → TerminalLosKontrolcu`

`Cube LOCAL_POSITION_NED/ATTITUDE → kendi araç durumu ve güvenlik kapıları`

Hedef konumu, hızı ve yönü kontrolcüye veya `ref_*` log alanlarına verilmez.

## Zorunlu kapılar

- Canlı koşuda `--menzil-kaynak sabit` kod tarafından reddedilir.
- Canlı angajman için araç `GUIDED`, heartbeat taze, `LOCAL_POSITION_NED` taze ve gerçek menzil taze olmalıdır.
- Durum 0,50 s, heartbeat 2,0 s veya menzil 2,0 s bayatlarsa komut gönderilmeden yetki bırakılır.
- Hedef ve avcı aynı sahada home alıyorsa `--target-alt-kaynagi relative`; farklı home irtifalarında ve güvenilir AMSL ile `amsl` kullanılır.

## Tek satır komutlar

Microhard hedef telemetrisini Pi'ye yönlendir: `mavproxy.py --master=<MICROHARD_HEDEF_SERI>,<BAUD> --out=udp:<PI_IP>:14604 --streamrate=10 --source-system=253 --non-interactive --no-state`

Balon menzil + gerçek görüntü dry-run: `python3 -u goruntulu_gudum.py --gudum los --buyuk-kare 5 --alan-pct 3 --dry-run --no-yetki-yaz --no-statustext --pursuer udpin:127.0.0.1:14552 --menzil-kaynak telemetri --target udpin:0.0.0.0:14604 --target-alt-kaynagi relative --log logs/balon_dry.csv`

Yarışma estimator dry-run: `python3 -u goruntulu_gudum.py --gudum los --buyuk-kare 5 --alan-pct 3 --dry-run --no-yetki-yaz --no-statustext --pursuer udpin:127.0.0.1:14552 --menzil-kaynak estimator --target udpin:0.0.0.0:14604 --log logs/yarisma_estimator_dry.csv`

Canlı komut, ancak dry-run logu ve preflight kapıları doğrulandıktan sonra aynı komuttan `--dry-run --no-yetki-yaz --no-statustext` çıkarılarak açılır.
