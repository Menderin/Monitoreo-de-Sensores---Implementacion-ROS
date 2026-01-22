"""Componentes de gráficos con Plotly"""
import plotly.graph_objects as go
from config.colors import COLORS

def crear_grafico_lineas(df, columna, titulo, color_key, yaxis_title):
    """
    Crea un gráfico de líneas con Plotly
    
    Args:
        df: DataFrame con los datos
        columna: Nombre de la columna a graficar
        titulo: Título del gráfico
        color_key: Clave de color en COLORS
        yaxis_title: Título del eje Y
        
    Returns:
        Figura de Plotly
    """
    fig = go.Figure()
    
    fig.add_trace(go.Scatter(
        x=df['timestamp'],
        y=df[columna],
        mode='lines+markers',
        name=titulo,
        line=dict(color=COLORS[color_key], width=3),
        marker=dict(size=6, color=COLORS[color_key]),
        fill='tozeroy',
        fillcolor=f'rgba{tuple(list(int(COLORS[color_key][i:i+2], 16) for i in (1, 3, 5)) + [0.2])}'
    ))
    
    # Calcular rango dinámico del eje Y
    y_min = df[columna].min() - 0.5
    y_max = df[columna].max() + 0.5
    
    fig.update_layout(
        title=dict(
            text=titulo,
            font=dict(color='#FFFFFF', size=25),
            x=0.5,
            xanchor='center'
        ),
        xaxis_title="Tiempo",
        yaxis_title=yaxis_title,
        height=350,  # Reducido para minimizar scroll innecesario
        plot_bgcolor='rgba(0,0,0,0)',
        paper_bgcolor='rgba(0,0,0,0)',
        font=dict(color='white', size=12),
        margin=dict(b=100, t=80, l=60, r=40),  # Margen inferior aumentado a 100px
        xaxis=dict(
            gridcolor='rgba(255,255,255,0.1)',
            title_font=dict(color='white', size=16),
            tickfont=dict(color='white', size=14),
            dtick=60000,  # Intervalo de 1 minuto
            tickformat="%H:%M:%S",
            showticklabels=True,  # Asegurar que los labels están visibles
            side='bottom'  # Forzar que el eje esté en la parte inferior
        ),
        yaxis=dict(
            gridcolor='rgba(255,255,255,0.1)',
            title_font=dict(color='white', size=16),
            tickfont=dict(color='white', size=14),
            range=[y_min, y_max]
        ),
        hovermode='x unified'
    )
    
    return fig

def crear_grafico_caja(df, columna, titulo, color_key, yaxis_title):
    """
    Crea un gráfico de caja (box plot) con Plotly
    
    Args:
        df: DataFrame con los datos
        columna: Nombre de la columna a graficar
        titulo: Título del gráfico
        color_key: Clave de color en COLORS
        yaxis_title: Título del eje Y
        
    Returns:
        Figura de Plotly
    """
    fig = go.Figure()
    
    fig.add_trace(go.Box(
        y=df[columna],
        name=titulo,
        marker_color=COLORS[color_key],
        boxmean='sd'
    ))
    
    fig.update_layout(
        height=300,
        plot_bgcolor='rgba(0,0,0,0)',
        paper_bgcolor='rgba(0,0,0,0)',
        font=dict(color='white'),
        yaxis=dict(
            title=yaxis_title,
            gridcolor='rgba(255,255,255,0.1)',
            title_font=dict(color='white'),
            tickfont=dict(color='white')
        ),
        showlegend=False
    )
    
    return fig
