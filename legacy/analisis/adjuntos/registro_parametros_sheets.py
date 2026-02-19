#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sistema de Registro de Parámetros pH y Temperatura - Google Sheets
Autores: Victor y Martin
"""

import gspread
from google.oauth2.service_account import Credentials
from datetime import datetime
import os


class RegistroParametrosGoogleSheets:
    def __init__(self, credentials_file='credentials.json', sheet_id=None):
        """
        Inicializa el sistema de registro con Google Sheets
        
        Args:
            credentials_file: Archivo JSON con las credenciales de la cuenta de servicio
            sheet_id: ID de la hoja de Google Sheets (desde la URL)
        """
        self.credentials_file = credentials_file
        self.sheet_id = sheet_id
        self.client = None
        self.sheet = None
        self.worksheet = None
        
        # Scopes necesarios para Google Sheets
        self.scopes = [
            'https://www.googleapis.com/auth/spreadsheets',
            'https://www.googleapis.com/auth/drive'
        ]
        
        self.conectar()
    
    def conectar(self):
        """Establece conexión con Google Sheets"""
        try:
            # Verificar que exista el archivo de credenciales
            if not os.path.exists(self.credentials_file):
                print(f"\n❌ Error: No se encontró el archivo '{self.credentials_file}'")
                print("📖 Por favor lee la guía de configuración para obtener tus credenciales.")
                exit(1)
            
            # Autenticar con Google
            print("🔐 Autenticando con Google Sheets...")
            creds = Credentials.from_service_account_file(
                self.credentials_file,
                scopes=self.scopes
            )
            self.client = gspread.authorize(creds)
            
            # Si no se proporcionó un sheet_id, pedirlo
            if not self.sheet_id:
                print("\n" + "="*60)
                print("📊 CONFIGURACIÓN INICIAL")
                print("="*60)
                print("\nNecesitas el ID de tu Google Sheet.")
                print("Lo puedes encontrar en la URL de tu hoja:")
                print("https://docs.google.com/spreadsheets/d/[ESTE_ES_EL_ID]/edit")
                print("\n" + "-"*60)
                self.sheet_id = input("\n👉 Ingresa el ID de tu Google Sheet: ").strip()
                
                # Guardar el ID para próximas ejecuciones
                with open('.sheet_config', 'w') as f:
                    f.write(self.sheet_id)
                print("\n✅ Configuración guardada para futuras ejecuciones\n")
            
            # Abrir la hoja de cálculo
            print(f"📊 Conectando a Google Sheet...")
            self.sheet = self.client.open_by_key(self.sheet_id)
            
            # Usar la primera hoja o crear una nueva
            try:
                self.worksheet = self.sheet.sheet1
            except:
                self.worksheet = self.sheet.add_worksheet(title="Registros", rows="1000", cols="5")
            
            # Inicializar encabezados si la hoja está vacía
            self.inicializar_encabezados()
            
            print("✅ Conexión exitosa con Google Sheets\n")
            
        except gspread.exceptions.SpreadsheetNotFound:
            print("\n❌ Error: No se pudo acceder a la hoja de Google Sheets")
            print("📖 Asegúrate de:")
            print("   1. Haber compartido la hoja con la cuenta de servicio")
            print("   2. Que el ID de la hoja sea correcto")
            exit(1)
        except gspread.exceptions.APIError as e:
            print(f"\n❌ Error de API de Google: {e}")
            print("📖 Verifica que hayas habilitado Google Sheets API y Google Drive API")
            exit(1)
        except Exception as e:
            print(f"\n❌ Error inesperado: {e}")
            exit(1)
    
    def inicializar_encabezados(self):
        """Crea los encabezados si la hoja está vacía"""
        try:
            # Verificar si ya hay encabezados
            primera_fila = self.worksheet.row_values(1)
            
            if not primera_fila or primera_fila[0] != 'Fecha':
                print("📝 Inicializando encabezados en Google Sheets...")
                encabezados = ['Fecha', 'Hora', 'Usuario', 'pH', 'Temperatura (°C)']
                self.worksheet.update('A1:E1', [encabezados])
                
                # Formatear encabezados (opcional - negrita y color de fondo)
                self.worksheet.format('A1:E1', {
                    "backgroundColor": {"red": 0.267, "green": 0.447, "blue": 0.769},
                    "textFormat": {"bold": True, "foregroundColor": {"red": 1, "green": 1, "blue": 1}},
                    "horizontalAlignment": "CENTER"
                })
                
                print("✅ Encabezados creados correctamente\n")
        except Exception as e:
            print(f"⚠️  Advertencia al inicializar encabezados: {e}")
    
    def validar_ph(self, valor):
        """
        Valida que el pH esté en el rango válido (0-14)
        
        Args:
            valor: Valor de pH a validar
            
        Returns:
            float o None si es inválido
        """
        try:
            ph = float(valor)
            if 0 <= ph <= 14:
                return ph
            else:
                print("⚠️  Error: El pH debe estar entre 0 y 14")
                return None
        except ValueError:
            print("⚠️  Error: Ingrese un número válido")
            return None
    
    def validar_temperatura(self, valor):
        """
        Valida que la temperatura sea un número razonable (-50 a 150 °C)
        
        Args:
            valor: Valor de temperatura a validar
            
        Returns:
            float o None si es inválido
        """
        try:
            temp = float(valor)
            if -50 <= temp <= 150:
                return temp
            else:
                print("⚠️  Error: La temperatura debe estar entre -50°C y 150°C")
                return None
        except ValueError:
            print("⚠️  Error: Ingrese un número válido")
            return None
    
    def agregar_registro(self, usuario, ph, temperatura):
        """
        Agrega un nuevo registro a Google Sheets
        
        Args:
            usuario: Nombre del usuario (Victor o Martin)
            ph: Valor de pH
            temperatura: Valor de temperatura
        """
        try:
            # Obtener fecha y hora actual
            ahora = datetime.now()
            fecha = ahora.strftime('%Y-%m-%d')
            hora = ahora.strftime('%H:%M:%S')
            
            # Crear nueva fila
            nueva_fila = [fecha, hora, usuario, ph, temperatura]
            
            # Agregar a Google Sheets
            print("☁️  Guardando en Google Sheets...")
            self.worksheet.append_row(nueva_fila)
            
            print(f"\n✅ Registro guardado exitosamente en la nube:")
            print(f"   📅 Fecha: {fecha}")
            print(f"   ⏰ Hora: {hora}")
            print(f"   👤 Usuario: {usuario}")
            print(f"   🧪 pH: {ph}")
            print(f"   🌡️  Temperatura: {temperatura}°C")
            print(f"   ☁️  Estado: Sincronizado con Google Sheets\n")
            
        except gspread.exceptions.APIError as e:
            print(f"\n❌ Error de API al guardar: {e}")
            print("⚠️  Verifica tu conexión a internet\n")
        except Exception as e:
            print(f"\n❌ Error inesperado al guardar el registro: {e}\n")
    
    def ingresar_parametros(self, usuario):
        """
        Solicita al usuario los parámetros de pH y temperatura
        
        Args:
            usuario: Nombre del usuario registrando los datos
        """
        print(f"\n{'='*50}")
        print(f"  📊 REGISTRO DE PARÁMETROS - {usuario.upper()}")
        print(f"{'='*50}\n")
        
        # Solicitar pH
        ph = None
        while ph is None:
            valor_ph = input("🧪 Ingrese el valor de pH (0-14): ")
            ph = self.validar_ph(valor_ph)
        
        # Solicitar Temperatura
        temperatura = None
        while temperatura is None:
            valor_temp = input("🌡️  Ingrese la temperatura (°C): ")
            temperatura = self.validar_temperatura(valor_temp)
        
        # Guardar registro
        self.agregar_registro(usuario, ph, temperatura)
    
    def mostrar_menu(self):
        """Muestra el menú principal del sistema"""
        print("\n" + "="*50)
        print("  🔬 SISTEMA DE REGISTRO DE PARÁMETROS")
        print("  ☁️  Guardando en Google Sheets")
        print("="*50)
        print("\n  1️⃣  Ingresar parámetros (Victor)")
        print("  2️⃣  Ingresar parámetros (Martin)")
        print("  3️⃣  Salir")
        print("\n" + "-"*50)
    
    def ejecutar(self):
        """Ejecuta el loop principal del programa"""
        print("\n🚀 Iniciando Sistema de Registro de Parámetros (Google Sheets)...\n")
        
        while True:
            self.mostrar_menu()
            opcion = input("\n👉 Seleccione una opción (1-3): ").strip()
            
            if opcion == '1':
                self.ingresar_parametros('Victor')
            elif opcion == '2':
                self.ingresar_parametros('Martin')
            elif opcion == '3':
                print("\n👋 ¡Hasta luego! Gracias por usar el sistema.")
                print("☁️  Todos tus datos están seguros en Google Sheets\n")
                break
            else:
                print("\n⚠️  Opción inválida. Por favor seleccione 1, 2 o 3.\n")


def main():
    """Función principal"""
    # Verificar si existe configuración guardada
    sheet_id = None
    if os.path.exists('.sheet_config'):
        with open('.sheet_config', 'r') as f:
            sheet_id = f.read().strip()
    
    # Crear instancia del sistema
    sistema = RegistroParametrosGoogleSheets(
        credentials_file='credentials.json',
        sheet_id=sheet_id
    )
    
    # Ejecutar el programa
    sistema.ejecutar()


if __name__ == "__main__":
    main()
