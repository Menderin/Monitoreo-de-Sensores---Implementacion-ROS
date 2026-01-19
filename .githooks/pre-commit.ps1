# ============================================
# PRE-COMMIT HOOK: Detectar archivos .env
# ============================================
# Hook para PowerShell en Windows

Write-Host "[CHECKING] Verificando archivos sensibles..." -ForegroundColor Cyan

# Buscar archivos .env en el staged area
$envFiles = git diff --cached --name-only | Select-String -Pattern "\.env(\.|$)"

if ($envFiles) {
    Write-Host ""
    Write-Host "[ERROR] Intentaste commitear archivo(s) .env:" -ForegroundColor Red
    $envFiles | ForEach-Object { Write-Host "   - $_" -ForegroundColor Yellow }
    Write-Host ""
    Write-Host "[INFO] Los archivos .env contienen credenciales y NO deben commitearse." -ForegroundColor Yellow
    Write-Host "       Usa .env.example para plantillas sin credenciales." -ForegroundColor Yellow
    Write-Host ""
    exit 1
}

# Buscar otros archivos sensibles
$sensitiveFiles = git diff --cached --name-only | Select-String -Pattern "(secrets\.json|credentials\.json|\.pem|\.key)$"

if ($sensitiveFiles) {
    Write-Host ""
    Write-Host "[WARNING] Archivos potencialmente sensibles detectados:" -ForegroundColor Yellow
    $sensitiveFiles | ForEach-Object { Write-Host "   - $_" -ForegroundColor Yellow }
    Write-Host ""
    
    $confirm = Read-Host "Estas seguro de que quieres commitear estos archivos? (y/N)"
    if ($confirm -ne "y" -and $confirm -ne "Y") {
        Write-Host "[CANCELLED] Commit cancelado." -ForegroundColor Red
        exit 1
    }
}

Write-Host "[OK] Verificacion completada. Procediendo con el commit..." -ForegroundColor Green
exit 0
