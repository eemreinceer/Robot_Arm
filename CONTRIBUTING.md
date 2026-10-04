# Katkı Rehberi

Bu depo robotik yazılım, firmware ve fiziksel sistem kanıtlarını birlikte
taşır. Bir değişikliğin kabul edilebilir olması için kodun çalışması kadar
ölçüm sınıfı, güvenlik sınırı ve yeniden üretilebilirlik de açık olmalıdır.

## Çalışmaya başlamadan önce

1. Hedef branch'in güncel ve worktree'nin temiz olduğunu doğrulayın.
2. Açık issue'larda aynı problem için arama yapın.
3. Değiştireceğiniz paketin `package.xml`, build dosyaları, README ve testlerini
   inceleyin.
4. Fiziksel robot, servo rayı, UART veya firmware etkileniyorsa mevcut güvenlik
   ve bring-up belgelerini okuyun.

## Değişiklik disiplini

- Kapsamı küçük tutun; ilgisiz refactor veya dependency eklemeyin.
- ROS 2 interface değişikliklerini publisher/subscriber/service/action
  tüketicileriyle birlikte değerlendirin.
- Simülasyon, mock test, donanımlı entegrasyon ve fiziksel ölçümü birbirinin
  yerine kullanmayın.
- Ölçüm iddiasında komut, ortam, sürüm, tarih ve artifact yolunu kaydedin.
- Secret, credential, kişisel ağ adresi, yerel kullanıcı adı veya makineye özel
  mutlak yol commit etmeyin.

## Doğrulama

En küçük ortak kapı:

```bash
./scripts/verify_workspace.sh --quick
```

ROS 2 paketleri veya native firmware değiştiyse:

```bash
./scripts/verify_workspace.sh --full
```

Ek olarak değişen paketin dar testlerini çalıştırın. Test başarısızlığını veya
çalıştırılamayan fiziksel adımı teslim notunda açıkça belirtin.

## Commit ve pull request

- Küçük ve mantıksal commitler kullanın.
- Commit mesajında değişikliğin sonucunu açıklayın; yalnız dosya adını yazmayın.
- Force-push ve history rewrite kullanmayın.
- Pull request; problem, çözüm, risk, test sonucu ve geri alma yaklaşımını
  içermelidir.

## Simülasyon ve fiziksel güvenlik

- Simülasyon yalnız `./start_simulation.sh <mod>` ile açılır.
- Aynı anda tek simülasyon çalışır ve iş sonunda bütün çocuk prosesler kapatılır.
- Logdaki başarı metni fiziksel sim durumu değildir; pose verisi ve görsel durum
  birlikte doğrulanır.
- Robot hareketi, ray enerjisi, firmware flash veya kablolama değişikliği için
  operatör kontrollü preflight ve erişilebilir fiziksel güç kesme yolu gerekir.
- `/joint_states` açık çevrim komut yankısıysa ölçülmüş servo konumu değildir.

## Tamamlanma ölçütü

Bir katkı; kabul kriterlerini karşılıyor, ilgili testleri geçiyor,
`git diff --check` temiz, dokümantasyon güncel ve fiziksel kanıt sınırı doğru
ifade edilmişse teslim edilmiş sayılır.
