# modbus-alarm-logger

Sahadaki bir cihazdan Modbus TCP ile veri okuyup, değerler belirlenen sınırların dışına çıktığında alarm üreten küçük ama eksiksiz bir izleme sistemi. Gerçek bir PLC ya da sensör olmadan denenebilsin diye projeye kendi sanal Modbus cihazını da ekledim. Programı çalıştırdığınızda bu sanal cihaz arka planda ayağa kalkıyor, alarm istemcisi ona bağlanıyor ve olan biteni konsoldan canlı olarak izleyebiliyorsunuz.

Kod tamamen C++17 ile yazıldı ve standart kütüphane dışında hiçbir bağımlılığı yok. Windows (MinGW / MSVC), Linux ve macOS üzerinde derlenecek şekilde tasarlandı.

## Neden böyle bir proje?

Endüstriyel tesislerde sıcaklık, basınç, motor devri gibi değerler çoğunlukla Modbus üzerinden okunur. SCADA ve HMI yazılımlarının temelinde de aslında hep aynı döngü vardır: cihaza düzenli aralıklarla sor, gelen ham değeri anlamlı birime çevir, sınırlarla karşılaştır, bir şey ters gidiyorsa haber ver. Bu proje o döngüyü en sade ama gerçekçi hâliyle ortaya koymak için yazıldı. Protokolü hazır bir kütüphaneye bırakmak yerine çerçeveleri bayt bayt kendim oluşturup çözümledim, böylece arka planda neler döndüğü kodu okuyan herkes için açıkça görülebiliyor.

## Modbus'a kısa bir bakış

Modbus, 1979'da Modicon tarafından geliştirilmiş, sade yapısı sayesinde bugün hâlâ en yaygın kullanılan endüstriyel haberleşme protokollerinden biri. İşleyiş istemci–sunucu (eski adıyla master–slave) mantığına dayanır: istemci bir istek gönderir, cihaz da buna cevap verir. Cihaz kendiliğinden veri göndermez.

Cihazdaki veriler "register" adı verilen 16 bitlik hücrelerde tutulur. Bu projede kullanılan **Holding Register** alanı, genellikle ölçüm ve ayar değerlerinin bulunduğu okunup yazılabilir bölgedir. Okuma işlemi **0x03 (Read Holding Registers)** fonksiyon koduyla yapılır.

Modbus TCP'de her mesajın başında 7 baytlık bir **MBAP başlığı** bulunur:

| Alan | Boyut | Açıklama |
|---|---|---|
| Transaction ID | 2 bayt | İstek ile cevabı eşleştirmek için kullanılan sıra numarası |
| Protocol ID | 2 bayt | Modbus için her zaman 0 |
| Length | 2 bayt | Kendisinden sonra gelen bayt sayısı |
| Unit ID | 1 bayt | Hedef cihazın adresi |

Başlığın ardından asıl işi tanımlayan **PDU** gelir. Örneğin 0 numaralı adresten başlayarak 5 register okumak isteyen bir istemcinin gönderdiği çerçeve şöyle görünür:

```
00 01  00 00  00 06  01  03  00 00  00 05
 TxID  Proto  Len   Unit FC  Başlangıç Adet
```

Tüm 16 bitlik alanlar big-endian (yüksek bayt önce) gönderilir. Cihaz isteği yerine getiremediğinde fonksiyon kodunun en üst bitini 1 yaparak (0x83) bir **exception** cevabı döner. "Geçersiz adres" ya da "desteklenmeyen fonksiyon" gibi durumlar bu şekilde bildirilir.

Register'lar yalnızca tam sayı tuttuğu için ondalıklı değerler genellikle ölçeklenerek gönderilir. Sanal cihaz da sahadaki gerçek cihazlar gibi davranır: 72,4 °C'lik bir sıcaklık register'a 724 olarak yazılır, istemci de bu değeri 10'a bölerek gerçek değeri bulur.

## Mimari

```
┌──────────────────────────── modbus-alarm-logger ────────────────────────────┐
│                                                                             │
│  ┌──────────── VirtualModbusServer ───────────┐                             │
│  │  simülasyon thread'i   →  register bankası │                             │
│  │  ağ thread'i (TCP 1502) ← istek / cevap →  │◄──┐                         │
│  └────────────────────────────────────────────┘   │ Modbus TCP              │
│                                                   │ (127.0.0.1)             │
│  ┌──────────── ModbusAlarmClient ─────────────┐   │                         │
│  │  polling thread'i  ────────────────────────┼───┘                         │
│  │  ölçekleme → eşik kontrolü → olay üretimi  │──► alarms.csv               │
│  └────────────────────────────────────────────┘──► konsol (callback)        │
│                                                                             │
│  main.cpp: ikisini başlatır, periyodik durum tablosunu ekrana basar         │
└─────────────────────────────────────────────────────────────────────────────┘
```

Sunucu ile istemci aynı process içinde çalışsalar da birbirleriyle yalnızca gerçek bir TCP soketi üzerinden konuşur. Yani istemciyi hiçbir değişiklik yapmadan gerçek bir cihaza yönlendirmek mümkün.

### Dosya yapısı

```
├── CMakeLists.txt
├── include/
│   ├── ModbusProtocol.hpp       MBAP başlığı, çerçeve oluşturma, exception tipleri
│   ├── TcpSocket.hpp            Platformdan bağımsız RAII soket sarmalayıcısı
│   ├── VirtualModbusServer.hpp  Sanal cihaz ve register haritası
│   └── ModbusAlarmClient.hpp    Alarm istemcisi, eşik kuralları, olay yapıları
├── src/
│   ├── TcpSocket.cpp
│   ├── VirtualModbusServer.cpp
│   ├── ModbusAlarmClient.cpp
│   └── main.cpp                 Komut satırı uygulaması
└── tests/
    └── ModbusTests.cpp          Birim ve uçtan uca testler
```

### Sanal cihazın register haritası

| Adres | Değişken | Ölçek | Birim |
|---|---|---|---|
| 0 | Proses sıcaklığı | ×10 | °C |
| 1 | Hat basıncı | ×100 | bar |
| 2 | Pompa motor devri | ×1 | rpm |
| 3 | Rulman titreşimi | ×100 | mm/s |
| 4 | Tank seviyesi | ×10 | % |
| 5 | Heartbeat sayacı | – | her adımda +1 |
| 6–15 | Rezerve | – | 0 |

Simülasyon her değeri üç bileşenin toplamı olarak üretir: yavaş değişen sinüs biçiminde bir salınım, küçük bir ölçüm gürültüsü ve ara sıra ortaya çıkan "proses bozulmaları". Bozulma başladığında değer birkaç saniye içinde hedefine doğru yumuşak bir şekilde tırmanır, bir süre orada kalır ve sonra yine yavaşça normale döner. Bu sayede alarmlar gerçek bir tesiste olduğu gibi önce uyarı, ardından alarm şeklinde kademeli olarak oluşur.

## Alarm mekanizması

Her kanal için dört sınır tanımlanabilir. Bunlardan herhangi biri boş bırakılabilir:

```
   LOW ALARM     LOW WARNING        NORMAL         HIGH WARNING    HIGH ALARM
───────┼──────────────┼───────────────────────────────────┼──────────────┼──────►
      2.5            3.2                                  5.8            6.5   bar
```

Kanalın durumu `NORMAL → WARNING → ALARM` şeklinde ilerleyen basit bir durum makinesi ile takip edilir. Bir olay **yalnızca durum değiştiğinde** üretilir. Değer 10 dakika boyunca alarm bölgesinde kalsa bile log'a tek bir kayıt düşer, normale döndüğünde ise bir kayıt daha eklenir. Operatörü aynı uyarıyla boğmamak için önemli olan da bu.

### Histerezis

Gürültülü bir sinyal sınırın hemen etrafında dolaşırken alarm saniyede birkaç kez açılıp kapanabilir. Sahada bu duruma "chattering" denir. Bunu önlemek için her kurala bir **histerezis** bandı tanımlanır:

- Bir duruma **girmek** için değerin sınırı geçmesi gerekir.
- O durumdan **çıkmak** için değerin sınırdan histerezis kadar geriye çekilmesi gerekir.

Örneğin sıcaklıkta yüksek alarm sınırı 85 °C, histerezis 1 °C ise alarm 85,0 °C'de devreye girer ama ancak 84,0 °C'nin altına inildiğinde kalkar. Karar mantığı `evaluateThreshold()` adlı, yan etkisi olmayan küçük bir fonksiyonda toplandığı için tek başına kolayca test edilebiliyor.

### Haberleşme alarmı

Cihaza ulaşılamaması da en az yüksek sıcaklık kadar önemli bir durum. Bağlantı koptuğunda, zaman aşımı yaşandığında ya da bozuk bir çerçeve geldiğinde istemci `LINK` kaynaklı bir alarm üretir, soketi kapatır ve bir sonraki turda yeniden bağlanmayı dener. Bağlantı geri geldiğinde de "Communication restored" kaydı düşülür. Cihaz isteğe Modbus exception ile cevap verirse bağlantı sağlam kabul edilir, bu durumda yalnızca hata sayacı artar.

### Log formatı

Tüm olaylar, Excel ya da herhangi bir veri aracıyla kolayca açılabilen bir CSV dosyasına eklenir. Her kayıttan sonra dosya diske yazıldığı için program beklenmedik şekilde kapansa bile alarm geçmişi kaybolmaz.

```csv
timestamp,source,previous,current,value,unit,message
2026-11-10 11:24:40.905,"Vibration",NORMAL,WARNING,5.43,"mm/s","HIGH WARNING: 5.43 mm/s (limit 4.50 mm/s)"
2026-11-10 11:24:42.459,"Vibration",WARNING,ALARM,7.48,"mm/s","HIGH ALARM: 7.48 mm/s (limit 7.10 mm/s)"
```

## Derleme

Gereksinimler: C++17 destekleyen bir derleyici (GCC 8+, Clang 7+, MSVC 2019+) ve CMake 3.16 veya üzeri.

**Linux / macOS**

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

**Windows (MinGW-w64)**

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

> Proje klasörünün yolunda Türkçe karakter ya da boşluk varsa `MinGW Makefiles` üreticisi dosyaları bulamayabiliyor. Ninja bu durumda sorunsuz çalışıyor.

**Windows (Visual Studio)**

```powershell
cmake -S . -B build
cmake --build build --config Release
```

Kullanılabilecek CMake seçenekleri:

| Seçenek | Varsayılan | Açıklama |
|---|---|---|
| `MAL_BUILD_TESTS` | `ON` | Test programını derler |
| `MAL_WARNINGS_AS_ERRORS` | `OFF` | Uyarıları hata olarak ele alır |
| `MAL_ENABLE_SANITIZERS` | `OFF` | AddressSanitizer ve UBSan ile derler (GCC/Clang, Linux/macOS) |

## Çalıştırma

```bash
./build/modbus-alarm-logger
```

Program sanal cihazı `127.0.0.1:1502` adresinde başlatır, istemciyi ona bağlar ve `Ctrl+C`'ye basılana kadar çalışır. Port olarak 502 yerine 1502 kullanılmasının sebebi, 1024'ün altındaki portların Linux'ta yönetici yetkisi istemesi.

| Parametre | Varsayılan | Açıklama |
|---|---|---|
| `--port <n>` | 1502 | Sanal cihazın portu (`0` verilirse boş bir port seçilir) |
| `--interval <ms>` | 500 | İstemcinin sorgulama aralığı |
| `--status <ms>` | 3000 | Durum tablosunun yenilenme aralığı |
| `--duration <s>` | 0 | Belirtilen süre sonunda kendiliğinden kapanır (0 = sınırsız) |
| `--log <dosya>` | alarms.csv | Alarm log dosyası |
| `--no-log` | – | Dosyaya log yazmayı kapatır |
| `--seed <n>` | rastgele | Simülasyonu tekrarlanabilir yapar |

Örnek bir çalışma:

```
modbus-alarm-logger | virtual device listening on 127.0.0.1:1502 (unit 1)
Polling every 500 ms, alarms logged to alarms.csv. Press Ctrl+C to stop.
[2026-11-10 11:24:40.905] WARNING  Vibration    HIGH WARNING: 5.43 mm/s (limit 4.50 mm/s)
[2026-11-10 11:24:42.459] ALARM    Vibration    HIGH ALARM: 7.48 mm/s (limit 7.10 mm/s)
[2026-11-10 11:24:44.523] WARNING  MotorSpeed   HIGH WARNING: 1648.00 rpm (limit 1600.00 rpm)
[2026-11-10 11:24:46.062] ALARM    MotorSpeed   HIGH ALARM: 1708.00 rpm (limit 1700.00 rpm)
------------------------------------------------------------------
 2026-11-10 11:24:46 | link ONLINE  | polls 32 (failed 0) | events 5
 device: 1 connection(s), 32 request(s) served
------------------------------------------------------------------
 CHANNEL            VALUE  UNIT  STATE
 Temperature        65.40  C     NORMAL
 Pressure            4.27  bar   NORMAL
 MotorSpeed       1708.00  rpm   ALARM
 Vibration           6.60  mm/s  WARNING
 TankLevel          65.00  %     NORMAL
------------------------------------------------------------------
```

Sanal cihaz standart bir Modbus TCP sunucusu olduğu için program çalışırken modbus-cli, QModMaster veya Modbus Poll gibi araçlarla da bağlanıp register'ları okuyabilirsiniz.

## Testler

```bash
ctest --test-dir build --output-on-failure
```

Testler herhangi bir dış framework gerektirmiyor. Kapsanan konular:

- MBAP başlığının kodlanıp çözülmesi ve istek çerçevesinin bayt düzeni
- Eşik seviyeleri ve histerezis davranışı
- Gerçek soket üzerinden uçtan uca alarm döngüsü (normal → alarm → normal)
- Geçersiz adres isteğinde Modbus exception işlenmesi
- Bağlantı koptuğunda tek bir `LINK` alarmı üretilmesi
- Arka plan thread'lerinin temiz bir şekilde durdurulması

## Tasarım notları

**Kaynak yönetimi.** Projede hiç `new` / `delete` kullanılmıyor. Soketler taşınabilir ama kopyalanamaz bir `TcpSocket` sınıfı içinde tutuluyor ve nesne kapsam dışına çıktığında otomatik olarak kapanıyor. Kodun hangi noktasında exception fırlatılırsa fırlatılsın açık kalan bir dosya tanımlayıcısı olmuyor. Windows'taki `WSAStartup` / `WSACleanup` çifti de `NetworkRuntime` adlı bir RAII nesnesine bağlı. `getaddrinfo` sonucu özel bir deleter ile `std::unique_ptr` içinde tutuluyor, log dosyası ise `std::ofstream` ile yönetiliyor.

**Thread'ler.** Sunucu ve istemci kendi thread'lerini kendileri yönetiyor ve hem `stop()` hem de yıkıcı fonksiyon içinde bu thread'leri `join` ediyor. Beklemeler `sleep` yerine `condition_variable::wait_until` ile yapıldığı için durdurma isteği anında karşılanıyor. Paylaşılan veriler (register bankası, durum özeti, log dosyası) ayrı mutex'lerle korunuyor. Callback ve log yazma işlemleri durum kilidi bırakıldıktan sonra yapıldığı için kilitlenme riski yok.

**Const-correctness.** Yapılandırmalar nesne oluşturulduktan sonra `const` olarak saklanıyor. Durumu değiştirmeyen tüm metotlar `const`, hata fırlatmayanlar ise `noexcept` olarak işaretlendi. Önemli dönüş değerlerinin göz ardı edilmemesi için `[[nodiscard]]` kullanıldı.

**Dayanıklılık.** Bozuk veya yarım kalmış bir istek sunucuyu çökertmiyor, sadece o bağlantı kapatılıyor. İstemci de transaction ID, protokol ID, uzunluk ve bayt sayısını tek tek doğruluyor. Beklenmeyen bir cevap gelirse akışın senkronunun kaybolduğunu varsayıp yeniden bağlanıyor.

Kod, GCC 16 ile `-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Werror` bayrakları altında hiç uyarı vermeden derleniyor. Bellek hatalarını ve sızıntıları kontrol etmek için Linux üzerinde şu komutlar kullanılabilir:

```bash
cmake -S . -B build-asan -DMAL_ENABLE_SANITIZERS=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build-asan && ctest --test-dir build-asan --output-on-failure
```

## Gerçek bir cihaza bağlamak

`ModbusAlarmClient` sanal cihaza özel hiçbir şey bilmiyor. Gerçek bir cihazı izlemek için `main.cpp` içindeki `buildChannels()` fonksiyonunda register adreslerini, ölçekleri ve sınırları cihazın dokümanına göre güncellemek, `clientConfig.host` ve `clientConfig.port` alanlarını da cihazın adresine yönlendirmek yeterli. Bu durumda sanal sunucuyu başlatan satırlar kaldırılabilir.

## Geliştirme fikirleri

- Kanal tanımlarını JSON veya YAML dosyasından okumak
- Input Register (0x04), Coil (0x01) ve 32 bit float desteği eklemek
- Alarmlar için onay (acknowledge) mekanizması ve gecikme süresi (on-delay) tanımlamak
- Olayları MQTT ya da e-posta ile iletmek
- Sanal cihaza aynı anda birden fazla istemci bağlanabilmesini sağlamak
