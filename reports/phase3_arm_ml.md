# Faz 3 Raporu - arm_ml Derin Ogrenme IK

**Tarih:** 2026-05-30
**Durum:** Tam dataset, uzun egitim ve final TorchScript model tamamlandi.

## Uygulama

- Dataset FK frame'i kanonik `Link_6`; `tool_link` offset'i dataset hattinda kullanilmiyor.
- Joint limitleri Python ve C++ tarafinda self-collision araliklariyla senkron:
  `j1 [-3.14, 3.14]`, `j2 [-2.86, 1.25]`, `j3 [-3.14, 0.94]`,
  `j4 [-3.14, 3.14]`, `j5 [-2.44, 2.33]`, `j6 [-3.14, 3.14]`.
- Quaternionlar giris uzayindaki `q/-q` sureksizligini azaltmak icin `w >= 0` kanonik forma getirildi.
- Dataset: `1_000_000` ornek, `.npz`, split `700_000 / 150_000 / 150_000`.
- Model: `IKNet`, `7 -> 256 -> 512 -> 512 -> 256 -> 6`, BatchNorm, ReLU, Dropout, Tanh.
- Loss: joint MSE + diferansiyellenebilir `Link_6` FK pozisyon reconstruction loss.
- Training: AdamW, cosine LR scheduler, TensorBoard logging, early stopping, checkpoint resume destegi.
- Export: `src/arm_ml/models/ik_net_scripted.pt` (`torch.jit.script`). Model ve
  dataset yerel artifact olarak kalır; public portfolio ağırlığı dağıtmaz.
- `evaluate_ik` komutu eklendi; train/val/test splitlerinde FK reconstruction mm metrikleri uretir.
- `/dl_ik_solve` servisi artik tahminle birlikte gercek FK reconstruction error degerini de dondurur.

## Tam Egitim

WSL ortami CPU-only: PyTorch `2.11.0+cpu`, CUDA yok, `16` CPU.

1. Temel uzun egitim: `100` epoch, `fk_lambda=10`, `58m26s`, en iyi val loss `0.246352`.
2. FK odakli refinement: resume, `fk_lambda=100`, `lr=1e-3`, early stopping epoch `38`, `15m01s`.
3. Stabil refinement: resume, `fk_lambda=100`, `lr=1e-4`, early stopping epoch `62`, `23m50s`.

Final checkpoint:

- `val_loss=0.303824` (`joint_mse + 100 * fk_position_mse`)
- `fk_lambda=100.0`
- Dataset boyutu: `96,401,023` byte
- Checkpoint boyutu: `2,149,349` byte
- TorchScript boyutu: `2,170,153` byte

## Final Test Metrikleri

`150_000` ayri test ornegi uzerinde `Link_6` FK reconstruction position error:

| Metrik | Deger |
| --- | --- |
| Mean | `16.648 mm` |
| Median | `13.998 mm` |
| P95 | `29.899 mm` |
| RMSE | `27.648 mm` |
| Max | `1607.815 mm` |
| TorchScript latency | `0.058 ms` ortalama (`1000` istek) |

## Dogrulama

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=src/arm_ml
python3 -m py_compile src/arm_ml/arm_ml/*.py src/arm_ml/scripts/*.py
python3 -m arm_ml.evaluate_model --split test --batch-size 2048
python3 -m arm_ml.benchmark_latency --requests 1000
colcon build --symlink-install --packages-select arm_ml
ros2 run arm_ml dl_ik_node
ros2 service call /dl_ik_solve arm_interfaces/srv/SolveIk ...
```

Son servis smoke testi:

- `/dl_ik_solve` basarili cevap verdi.
- Home `Link_6` hedefi icin inference `2.136 ms`.
- Servisin raporladigi gercek FK reconstruction error `12.774 mm`.

## Bilinen Sinirlar

- Ortalama hata ciddi sekilde dusuruldu ancak P95 `29.899 mm`; DL IK tek basina hassas pick son cozucusu olarak kabul edilmemeli.
- Max hata `1607.815 mm`; nadir outlier'lar var. Uretim hattinda joint-limit ve FK-error gate uygulanmali.
- Onerilen kullanim: DL model hizli seed uretir; analitik DLS IK veya MoveIt/pick_ik final refinement ve dogrulama yapar.
- Sonraki adim `benchmark_ik_comparison.py` ile canli 3-cozucu benchmark'idir.
