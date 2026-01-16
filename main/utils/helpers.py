"""Funciones auxiliares generales"""
import base64

def get_image_base64(image_path):
    """
    Convierte imagen a base64 para uso en HTML
    
    Args:
        image_path: Ruta a la imagen
        
    Returns:
        String base64 de la imagen o None si hay error
    """
    try:
        with open(image_path, "rb") as image_file:
            image_bytes = image_file.read()
        return base64.b64encode(image_bytes).decode()
    except:
        return None
