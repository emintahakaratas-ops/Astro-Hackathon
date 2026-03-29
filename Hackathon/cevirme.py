import rasterio
import numpy as np

def tiff_to_matrix(file_path):
    # 1. Dosyayı açıyoruz
    with rasterio.open(file_path) as src:
        # NASA verilerinde yükseklik genelde 1. kanaldadır (Band 1)
        height_matrix = src.read(1)
        
        # Eğer veride 'boş' veya 'hatalı' pikseller varsa onları 0 yapalım
        height_matrix = np.nan_to_num(height_matrix)
        
        # Satır (n) ve Sütun (m) sayılarını alalım
        n, m = height_matrix.shape
        
        print(f"Matris Boyutu: {n}x{m}")
        print(f"Örnek Yükseklik Verisi (0,0): {height_matrix[0,0]} metre")
        
        return height_matrix, n, m

# Kullanım:
matrix, n, m = tiff_to_matrix('test2231.tif')

# 3. C++ Kodun İçin Kaydetme (GÜNCELLENDİ)
# Dosyayı "yazma" modunda açıp önce n ve m'yi, sonra matrisi basıyoruz
with open("input_heights.txt", "w") as f:
    # İlk satıra n ve m yaz, altına geç
    f.write(f"{n} {m}\n")
    
    # Geri kalanına matrisi yapıştır
    np.savetxt(f, matrix, fmt="%.2f")

print("Vay be! n, m ve yükseklik verileri 'input_heights.txt' içine fıstık gibi işlendi.")
