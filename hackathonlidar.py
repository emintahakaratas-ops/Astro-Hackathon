 import numpy as np
import os

class AyAraciLidarSistemi:
    def __init__(self, dosya_yolu="heightmap.txt", harita_boyutu=500, cozunurluk=0.1):
        """
        dosya_yolu: NASA verilerinin olduğu .txt dosyası
        harita_boyutu: Arkadaşından gelecek (Örn: 500)
        cozunurluk: Arkadaşından gelecek (Örn: 0.1)
        """
        self.map_size = harita_boyutu
        self.res = cozunurluk
        
        # 1. NASA VERİSİNİ DOSYADAN YÜKLE
        # Dosya yoksa sıfır matrisi oluşturur, varsa yükler.
        self.height_map = self.dosyadan_harita_yukle(dosya_yolu)
        
        # 2. FİZİKSEL SABİTLER
        self.ROBOT_YARICAP = 0.4     # 40cm içindeki noktalar robotun kendisidir
        self.CELISKI_ESIGI = 0.15    # NASA verisi ile Lidar arasında 15cm fark varsa güncelle
        
    def dosyadan_harita_yukle(self, dosya_yolu):
        """NASA'dan alınan heightmap.txt dosyasını okur."""
        if os.path.exists(dosya_yolu):
            try:
                # Dosyanın boşlukla ayrılmış sayılar içerdiğini varsayıyoruz
                yuklenen_veri = np.loadtxt(dosya_yolu)
                # Eğer dosyadaki boyut farklıysa yeniden boyutlandır veya hata ver
                if yuklenen_veri.shape != (self.map_size, self.map_size):
                    print(f"Uyarı: Dosya boyutu {yuklenen_veri.shape} beklenenle uyuşmuyor!")
                return yuklenen_veri
            except Exception as e:
                print(f"Hata: Dosya okunurken sorun çıktı: {e}")
                return np.zeros((self.map_size, self.map_size))
        else:
            print("Uyarı: heightmap.txt bulunamadı, boş harita oluşturuluyor.")
            return np.zeros((self.map_size, self.map_size))

    def metre_to_index(self, x_m, y_m):
        """Metre koordinatını dizi indeksine çevirir (Sol Alt 0,0)."""
        ix = int(x_m / self.res)
        iy = int(y_m / self.res)
        return ix, iy

    def haritayi_isleme_ve_guncelle(self, ham_lidar_xyz, robot_pos):
        """
        ham_lidar_xyz: [[x, y, z], [x, y, z]...] -> Lidar'dan gelen direkt x,y,z verisi
        robot_pos: [gx, gy, yaw] -> Robotun dünya koordinatı ve yönü
        """
        rx, ry, ryaw = robot_pos
        harita_degisti = False
        
        for nokta in ham_lidar_xyz:
            lx, ly, lz = nokta # Lidar'a göre x, y, z
            
            # --- ADIM 1: GÖVDE FİLTRESİ ---
            dist = np.sqrt(lx**2 + ly**2)
            if dist < self.ROBOT_YARICAP:
                continue
            
            # --- ADIM 2: LOKAL -> GLOBAL KOORDİNAT DÖNÜŞÜMÜ ---
            # Lidar x,y'sini robotun yönüne (yaw) ve konumuna göre Dünya'ya taşıyoruz
            gx = rx + (lx * np.cos(ryaw) - ly * np.sin(ryaw))
            gy = ry + (lx * np.sin(ryaw) + ly * np.cos(ryaw))
            
            # --- ADIM 3: DİZİ İNDEKSİNE YERLEŞTİRME ---
            ix, iy = self.metre_to_index(gx, gy)
            
            # Harita sınırları içindeyse kontrol et
            if 0 <= ix < self.map_size and 0 <= iy < self.map_size:
                nasa_z = self.height_map[ix, iy] # NASA'dan gelen eski değer
                lidar_z = lz # Lidar'dan gelen yeni değer
                
                # --- ADIM 4: NASA VERİSİYLE KARŞILAŞTIRMA ---
                # Eğer Lidar'ın gördüğü yükseklik, NASA verisinden çok farklıysa güncelle
                if abs(lidar_z - nasa_z) > self.CELISKI_ESIGI:
                    self.height_map[ix, iy] = lidar_z
                    harita_degisti = True
                    
        return harita_degisti, self.height_map

# --- KULLANIM ÖRNEĞİ ---
# 1. Sistemi başlat (Dosyayı otomatik okur)
# sistem = AyAraciLidarSistemi(dosya_yolu="heightmap.txt", harita_boyutu=500, cozunurluk=0.1)

# 2. Lidar'dan x,y,z verisi gelince (Örn: [[2.1, 0.5, -0.4], ...])
# degisti, yeni_map = sistem.haritayi_isleme_ve_guncelle(lidar_verisi, [10.0, 10.0, 0.0])

# 3. Eğer değişim varsa (Önemli bir fark bulunduysa)
# if degisti:
#     print("NASA verisinde olmayan bir engel/krater tespit edildi! Harita güncellendi.")