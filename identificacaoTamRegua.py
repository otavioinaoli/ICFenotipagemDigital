from paddleocr import PaddleOCR

def extrair_medida_cm(caminho_imagem):
    ocr = PaddleOCR(lang="en", enable_mkldnn=False)
    resultado = ocr.predict(caminho_imagem)
    
    numeros_encontrados = []
    for item in resultado:
        rec_texts = item.get('rec_texts', [])
        for texto in rec_texts:
            if texto.isdigit():
                numeros_encontrados.append(int(texto))
                
    return numeros_encontrados

medidas = extrair_medida_cm("regua_processada.jpg")
print("Valores detetados para cálculo:", medidas)