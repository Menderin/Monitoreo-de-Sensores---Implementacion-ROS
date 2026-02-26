"""Módulo de gestión de base de datos"""
from .mongo_handler import MongoHandler, cargar_todos_datos

__all__ = ['MongoHandler', 'cargar_todos_datos']
