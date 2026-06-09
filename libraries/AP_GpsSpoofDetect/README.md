# AP_GpsSpoofDetect — Detector de Spoofing GNSS para ArduCopter

## Índice

1. [¿Qué es el spoofing GNSS y por qué es peligroso?](#1-qué-es-el-spoofing-gnss-y-por-qué-es-peligroso)
2. [Contexto: cómo navega un dron con ArduPilot](#2-contexto-cómo-navega-un-dron-con-ardupilot)
3. [El EKF: el cerebro de la navegación](#3-el-ekf-el-cerebro-de-la-navegación)
4. [Por qué el EKF solo no es suficiente](#4-por-qué-el-ekf-solo-no-es-suficiente)
5. [Arquitectura del detector](#5-arquitectura-del-detector)
6. [Feature 1 — Divergencia de velocidad GPS vs EKF (CUSUM)](#6-feature-1--divergencia-de-velocidad-gps-vs-ekf-cusum)
7. [Feature 2 — Tendencia de varianzas del EKF](#7-feature-2--tendencia-de-varianzas-del-ekf)
8. [Feature 3 — Consistencia de aceleración GPS vs IMU](#8-feature-3--consistencia-de-aceleración-gps-vs-imu)
9. [Feature 4 — Inconsistencia de precisión declarada vs real](#9-feature-4--inconsistencia-de-precisión-declarada-vs-real)
10. [Fusión de features y máquina de estados](#10-fusión-de-features-y-máquina-de-estados)
11. [Parámetros configurables](#11-parámetros-configurables)
12. [Archivos modificados](#12-archivos-modificados)
13. [Proceso de validación y tuning](#13-proceso-de-validación-y-tuning)
14. [Cómo probar en SITL](#14-cómo-probar-en-sitl)
15. [Limitaciones conocidas y trabajo futuro](#15-limitaciones-conocidas-y-trabajo-futuro)

---

## 1. ¿Qué es el spoofing GNSS y por qué es peligroso?

### El GPS como vulnerabilidad

El sistema de posicionamiento global (GPS/GNSS) funciona recibiendo señales de satélites en órbita. El receptor en el dron mide el tiempo que tarda en llegar cada señal y, con al menos 4 satélites, calcula su posición y velocidad en el espacio.

**El problema fundamental**: las señales GPS son débiles (llegan desde ~20.000 km de altura) y no están cifradas en sus versiones civiles. Cualquier equipo de radio suficientemente potente puede generar señales GPS falsas que el receptor no distingue de las reales.

### Qué hace el atacante

Un atacante puede montar en tierra un transmisor que emite señales GPS falsas, ligeramente más potentes que las reales. El receptor del dron "se cree" las señales falsas y calcula una posición/velocidad incorrecta.

El ataque más peligroso es el **spoofing gradual**: el atacante no hace que el GPS salte de golpe (lo que el EKF rechazaría como outlier), sino que introduce un error pequeño que crece lentamente con el tiempo. El dron percibe que se está moviendo de forma "suave y natural" hacia donde el atacante quiere llevarlo.

### Consecuencias

- El dron abandona su zona de operación prevista sin que el operador lo note
- El dron puede ser capturado o estrellarse deliberadamente
- En aplicaciones críticas (entrega de material, vigilancia), el atacante controla el destino del vuelo

---

## 2. Contexto: cómo navega un dron con ArduPilot

ArduPilot es el software de autopiloto de código abierto más utilizado del mundo. La navegación de un dron se basa en fusionar información de varios sensores:

| Sensor | Qué mide | Frecuencia típica |
|---|---|---|
| **GPS** | Posición y velocidad absolutas | 5–20 Hz |
| **IMU (acelerómetro + giróscopo)** | Aceleraciones y velocidades angulares | 400–1000 Hz |
| **Barómetro** | Altitud relativa | 50 Hz |
| **Brújula (magnetómetro)** | Orientación magnética | 10–100 Hz |

Ningún sensor es perfecto por sí solo:
- El **GPS** da posición absoluta pero con ruido y latencia, y es vulnerable a spoofing
- La **IMU** es muy precisa a corto plazo pero acumula deriva (bias) con el tiempo
- El **barómetro** da altitud relativa pero se ve afectado por el viento

La solución es **fusionar todos los sensores** mediante un algoritmo matemático: el EKF.

---

## 3. El EKF: el cerebro de la navegación

### ¿Qué es un filtro de Kalman?

Imagina que quieres saber dónde está un dron en cada momento. Tienes dos tipos de información:

1. **Lo que predices** (basándote en lo que sabías antes + la física del movimiento)
2. **Lo que mides** (sensores como GPS, IMU, brújula)

Ambas tienen incertidumbre. El filtro de Kalman es el algoritmo matemático óptimo para combinar predicción y medición de forma ponderada, dando más peso a lo que tiene menor incertidumbre en cada instante.

### ¿Por qué "extendido"?

El filtro de Kalman clásico funciona solo con sistemas lineales. Un dron en 3D es altamente no lineal (rotaciones, aerodinámica, etc.). El **Extended Kalman Filter (EKF)** linealiza localmente el sistema en cada paso de tiempo, permitiendo aplicar la misma idea a sistemas complejos.

### Cómo funciona el EKF de ArduPilot paso a paso

ArduPilot implementa **EKF3** (tercera generación), que mantiene múltiples "lanes" (instancias paralelas del filtro) y elige la más saludable.

**Bucle de predicción** (a 400 Hz, con los datos de la IMU):
```
Estado(t) = F · Estado(t-1) + Q
```
Donde:
- `Estado` = vector de 24 variables: posición NED, velocidad NED, orientación (cuaternión), bias IMU, bias viento, bias brújula
- `F` = matriz de transición de estado (derivada de las ecuaciones de movimiento)
- `Q` = ruido del proceso (incertidumbre de la predicción)

A cada paso, la incertidumbre del estado **crece** (porque la predicción acumula error).

**Bucle de corrección** (cuando llega una medición GPS, a ~10 Hz):
```
K = P · H^T · (H · P · H^T + R)^-1    ← ganancia de Kalman
Estado = Estado + K · (z - H · Estado) ← corrección
P = (I - K · H) · P                    ← actualizar covarianza
```
Donde:
- `z` = medición GPS (posición y velocidad)
- `H` = matriz de observación (cómo conecta el estado con la medición)
- `R` = ruido de medición (confianza en el GPS)
- `K` = ganancia de Kalman (cuánto corregir)
- `P` = covarianza del estado (incertidumbre actual)

La **ganancia de Kalman K** es el corazón del filtro: si el GPS es muy fiable (R pequeño), K es grande y el filtro confía mucho en el GPS. Si el GPS es ruidoso (R grande), K es pequeño y el filtro confía más en la predicción de la IMU.

### Qué pasa cuando hay spoofing gradual

Cuando el atacante introduce un error de velocidad pequeño (por ejemplo, +0.1 m/s Norte cada segundo), el EKF lo interpreta como una medición GPS legítima con ruido bajo. El filtro **absorbe gradualmente el error** en su estimación de posición/velocidad.

El resultado: el EKF "cree" que el dron se está moviendo hacia el norte, y el autopiloto activa los motores para compensar... llevando el dron exactamente hacia donde el atacante quiere.

### Señales de que el EKF está sufriendo

Cuando el spoofing es suficientemente severo, el EKF emite estas señales visibles en los logs:

```
EKF3 lane switch 1       ← cambia a filtro de respaldo (lane 1)
EKF primary changed:1    ← confirma el cambio
GPS Glitch or Compass error  ← detecta inconsistencia GPS
EKF3 lane switch 0       ← puede volver al lane 0
Glitch cleared           ← da el glitch por superado
```

Estos mensajes indican que el EKF **detecta inconsistencias** pero no puede determinar si son spoofing, interferencias o fallo de hardware. Nuestro detector complementa al EKF con análisis estadístico específico para spoofing.

---

## 4. Por qué el EKF solo no es suficiente

El EKF tiene mecanismos de detección de outliers (rechaza mediciones que se desvían demasiado de la predicción). Sin embargo:

1. **El spoofing gradual elude el rechazo de outliers**: si el error crece 0.01 m/s por ciclo, nunca supera el umbral de "outlier" individual, pero se acumula durante minutos.

2. **El EKF no distingue origen del problema**: no sabe si el GPS está spoofed, interferido, o si hay un fallo de sensor. Puede cambiar de lane y seguir usando un GPS comprometido.

3. **El EKF no alerta al operador**: su trabajo es dar la mejor estimación posible, no generar alarmas.

4. **Con múltiples GPSs todos comprometidos**: si el atacante hace spoofing de todos los receptores, todos los lanes del EKF reciben datos erróneos.

El detector `AP_GpsSpoofDetect` es una **capa de seguridad adicional** que analiza estadísticamente la coherencia entre sensores para identificar patrones específicos de spoofing.

---

## 5. Arquitectura del detector

### Visión general

```
┌─────────────────────────────────────────────────────────────┐
│                      update() — 10 Hz                       │
│                                                             │
│  ┌─────────────┐   ┌──────────────────────────────────────┐ │
│  │   Fuentes   │   │           4 Features                 │ │
│  │   de datos  │   │                                      │ │
│  │             │   │  F1: GPS-EKF velocity divergence     │ │
│  │  GPS        │──▶│  F2: EKF variance trend             │ │
│  │  EKF(AHRS)  │──▶│  F3: GPS-IMU accel consistency      │ │
│  │  IMU        │──▶│  F4: Accuracy vs actual divergence  │ │
│  └─────────────┘   └──────────────┬───────────────────────┘ │
│                                   │                         │
│                    ┌──────────────▼───────────────────────┐ │
│                    │        Weighted fusion               │ │
│                    │     score = Σ(wᵢ × fᵢ) / Σwᵢ           │ │
│                    └──────────────┬───────────────────────┘ │
│                                   │                         │
│                    ┌──────────────▼───────────────────────┐ │
│                    │    State machine with hysteresis     │ │
│                    │  NOMINAL → SUSPICIOUS → CONFIRMED   │ │
│                    └──────────────┬───────────────────────┘ │
│                                   │                         │
│                    ┌──────────────▼───────────────────────┐ │
│                    │   GCS warnings + Dataflash logging    │ │
│                    └───────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### El ciclo de ejecución

El detector corre a **10 Hz** (cada 100 ms), sincronizado con el ciclo GPS. En cada ciclo:

1. Obtiene velocidad GPS raw (`v_gps`) y velocidad estimada por el EKF (`v_ekf`)
2. Obtiene métricas de precisión GPS (`hAcc`, `sAcc`)
3. Almacena el sample en un buffer circular (100 muestras = 10 segundos)
4. Calcula las 4 features estadísticas
5. Fusiona las features en un score global [0, 2]
6. Actualiza la máquina de estados
7. Emite avisos y logs

---

## 6. Feature 1 — Divergencia de velocidad GPS vs EKF (CUSUM)

### La idea

Esta es la feature principal y la que más evidencia genera. Se basa en que el GPS y el EKF deben proporcionar velocidades muy similares en condiciones normales. Cuando hay spoofing, el GPS reporta una velocidad falsa mientras que el EKF (basado principalmente en la IMU) mantiene una estimación más cercana a la realidad durante un tiempo.

**El residual de velocidad** es:
```
residual = v_GPS - v_EKF
```

En condiciones normales, este vector es pequeño (ruido de sensores, latencia del filtro). Bajo spoofing gradual, el GPS introduce un sesgo persistente que hace que el residual crezca y permanezca elevado.

### Por qué no basta con un umbral simple

Si simplemente comparamos `|residual| > umbral`, tendremos muchas falsas alarmas durante maniobras (el EKF tiene latencia en absorber cambios bruscos de velocidad) y muchas detecciones perdidas con spoofing suave.

La solución es el **CUSUM (Cumulative Sum Control Chart)**, un test estadístico diseñado para detectar cambios persistentes pero pequeños en una señal.

### Cómo funciona el CUSUM

El CUSUM acumula desviaciones sobre un nivel de referencia:

```
cusum[k] = max(0, cusum[k-1] + nu[k] - K)
```

Donde:
- `nu[k]` = residual normalizado en el instante k
- `K` = parámetro de referencia (slack, `GSPF_CUSUM_K`)
- La operación `max(0, ...)` reinicia el acumulador cuando la señal vuelve a la normalidad

**Normalización del residual**:
```
sigma = max(sAcc, 1.0)      ← precisión de velocidad GPS (mínimo 1 m/s)
nu = |v_GPS - v_EKF| / sigma
```

La normalización por `sAcc` hace que un mismo residual absoluto pese menos si el GPS declara una alta precisión (lo cual es coherente con que haya un problema real).

**El límite mínimo de sigma = 1.0** es una decisión de diseño importante: en SITL, el simulador reporta `sAcc = 0` (GPS perfecto), lo que causaría que residuales pequeños (~0.01 m/s) inflaran artificialmente el residual normalizado. Usando 1.0 como mínimo, el detector ignora residuales menores de 1.0 m/s cuando el GPS no reporta accuracy válida.

**El cap de CUSUM = 500**: si el vehículo pasa mucho tiempo bajo spoofing, el CUSUM podría acumular valores enormes. Sin un límite, cuando se quitara el spoofing, el detector permanecería en estado CONFIRMED durante mucho tiempo mientras decae. El cap de 500 garantiza que la recuperación sea rápida.

### Visualización del CUSUM

```
   nu ─┐
 3.0   │         ████████████
 2.0   │    ██████            ████
 1.5 ──┼────────────────────────────── ← K (slack)
 1.0   │
 0.5   │ ▓▓
 0.0 ──┴─────────────────────────────→ tiempo

CUSUM:
  0    ─────────────────────────────
       Sin spoofing: nu < K, CUSUM = 0

  50   │           ▄▄▄▄▄▄▄▄▄▄▄▄
  25   │      ▄▄▄▄▄            ▄▄▄
   0   ─────────────────────────────
       Con spoofing: nu > K, CUSUM crece
```

### Código relevante

```cpp
// Calcular residual y normalizarlo
Vector3f residual = v_gps - v_ekf;
float residual_norm = residual.length();
float sigma = fmaxf(sAcc, 1.0f);   // mínimo 1.0 m/s para robustez en SITL
float nu = residual_norm / sigma;

// Actualizar CUSUM
_cusum_pos[0] = fmaxf(0.0f, _cusum_pos[0] + nu - _cusum_k);
_cusum_pos[0] = fminf(_cusum_pos[0], 500.0f);  // cap para recuperación rápida

// Convertir a score [0, 2]
float ratio = _cusum_pos[0] / _innov_thresh;
_f1_score = fminf(ratio, 2.0f);
```

---

## 7. Feature 2 — Tendencia de varianzas del EKF

### La idea

El EKF no solo estima el estado del vehículo, sino también **cuánto confía** en esa estimación. Esta confianza se expresa como la covarianza del estado (matriz P), de la que se extraen varianzas para velocidad (`velVar`) y posición (`posVar`).

En condiciones normales, estas varianzas son bajas y estables:
- `velVar` nominal ≈ 0.5 m²/s²
- `posVar` nominal ≈ 2.0 m²

Cuando el GPS empieza a introducir datos inconsistentes (spoofing), el EKF detecta que las correcciones GPS no encajan bien con la predicción de la IMU. Esto se traduce en un **aumento gradual de las varianzas**: el filtro pierde confianza en su estimación.

Esta feature implementa un CUSUM secundario sobre la varianza normalizada:

```cpp
float velNorm = velVar / 0.5f;    // normalizar por valor nominal
float posNorm = posVar / 2.0f;
float avgNorm = (velNorm + posNorm) / 2.0f;
float nu = fmaxf(0.0f, avgNorm - 1.0f);  // solo acumula cuando > nominal
```

El CUSUM tiene un decaimiento más lento (`_cusum_k * 0.3f`) porque las varianzas del EKF responden más lentamente al spoofing.

### Limitación actual

En SITL, el EKF3 tiende a mantener varianzas muy bajas porque el simulador proporciona datos "perfectos". Esta feature tiene un peso de 0 en la configuración actual (pendiente de calibración en hardware real).

---

## 8. Feature 3 — Consistencia de aceleración GPS vs IMU

### La idea

El dron tiene dos formas independientes de medir su aceleración:

1. **Desde el GPS**: derivando numéricamente la velocidad GPS: `a_GPS = (v_GPS(t) - v_GPS(t-1)) / dt`
2. **Desde la IMU**: los acelerómetros miden directamente la aceleración en el cuerpo del vehículo, que se rota al frame NED y se corrige restando la gravedad

En condiciones normales, estas dos aceleraciones deben ser coherentes. Bajo spoofing, el GPS introduce aceleraciones falsas que no coinciden con lo que el IMU mide físicamente.

### Por qué hay que filtrar la aceleración GPS

Derivar numéricamente la velocidad GPS introduce mucho ruido de alta frecuencia. Por eso se aplica un filtro pasa-bajos simple:

```cpp
_a_gps_filt = _a_gps_filt * 0.7f + a_gps_raw * 0.3f;
```

Esto es una media exponencial con constante de tiempo ≈ 2-3 segundos, que suaviza las variaciones rápidas manteniendo las tendencias lentas.

### Corrección de la gravedad

La IMU mide la aceleración total incluyendo la reacción al campo gravitatorio. Para comparar con la aceleración GPS (que es relativa al suelo, sin gravedad), hay que sumar 9.81 m/s² en el eje Z (NED):

```cpp
a_imu_ned.z += GRAVITY_MSS;  // restar la gravedad del marco NED
```

Esto tiene un signo aparentemente contrario a la intuición, pero en el marco NED el eje Z apunta hacia abajo, así que sumar +9.81 en Z equivale a cancelar la componente gravitatoria hacia abajo.

### Limitación actual

Esta feature también tiene peso 0 en la configuración actual. La derivación numérica de la velocidad GPS tiene mucho ruido incluso en vuelo nominal, lo que provoca falsas alarmas. Sería necesaria una ventana de tiempo más larga o un filtro más agresivo para hacerla robusta.

---

## 9. Feature 4 — Inconsistencia de precisión declarada vs real

### La idea

El GPS no solo reporta posición y velocidad, sino también estimaciones de su precisión:
- `sAcc`: precisión de velocidad (m/s, 1-sigma)
- `hAcc`: precisión horizontal de posición (m, 1-sigma)

En condiciones normales, si el GPS dice que su velocidad tiene un error de ±0.3 m/s (sAcc=0.3), el residual `|v_GPS - v_EKF|` debería ser estadísticamente consistente con eso: si el residual es 2.0 m/s pero el GPS declara sAcc=0.3 m, eso es una **inconsistencia estadística de 6-sigma**.

Los atacantes de spoofing que intentan engañar al receptor GPS para que dé señales aparentemente "buenas" mantienen o reducen artificialmente los valores de `sAcc` y `hAcc`. Pero el residual real sigue siendo alto porque el EKF detecta la inconsistencia.

```cpp
float actual_diff = (v_gps - v_ekf).length();
float expected_diff = fmaxf(sAcc, 0.05f);

// GPS claims high accuracy but actual divergence is large → inconsistent
if (actual_diff > 3.0f * expected_diff && hAcc < 3.0f) {
    _cusum_pos[3] = fmaxf(0.0f, _cusum_pos[3] + 1.0f - _cusum_k);
```

### Limitación actual

Esta feature tiene peso 0 en la configuración actual porque en SITL el GPS reporta `sAcc=0` casi siempre, haciendo imposible calcular la inconsistencia. En hardware real, donde el GPS reporta valores de sAcc precisos, esta feature sería más útil.

---

## 10. Fusión de features y máquina de estados

### Fusión ponderada

Los scores individuales de las 4 features se combinan en un score global mediante media ponderada:

```
score = (W1·f1 + W2·f2 + W3·f3 + W4·f4) / (W1 + W2 + W3 + W4)
```

Cada feature tiene un peso ajustable (`GSPF_W1` a `GSPF_W4`). En la configuración actual:
- **W1 = 1.0**: solo F1 (divergencia de velocidad) contribuye al score
- **W2 = W3 = W4 = 0**: features deshabilitadas (calibración pendiente para hardware real)

Con F1 dominando, el score puede alcanzar hasta 2.0 (F1_max = 2.0 × W1/W_total = 2.0).

### Máquina de estados con histéresis

La máquina de estados convierte el score continuo en un estado discreto con **tres niveles** y **histéresis temporal** para evitar oscilaciones rápidas (flapping):

```
                    score > TH_LOW          score >= TH_HIGH
                    durante 0.5s             durante 3s
  NOMINAL ──────────────────────▶ SUSPICIOUS ──────────────▶ CONFIRMED
    ▲                                   │                       │
    │           score < TH_LOW×0.5      │                       │
    └───────────────────────────────────┘                       │
    │                                                           │
    │                      score < TH_LOW×0.3                   │
    └───────────────────────────────────────────────────────────┘
                              durante 2s
```

**Detalles de cada transición:**

#### NOMINAL → SUSPICIOUS
- Condición: `score > GSPF_TH_LOW (0.5)` durante **5 ciclos consecutivos** (0.5 s)
- Propósito: Evitar que un spike momentáneo cause alarma

#### SUSPICIOUS → CONFIRMED
- Condición: `score >= GSPF_TH_HIGH (1.0)` durante **30 ciclos consecutivos** (3 s)
- Propósito: Asegurarse de que el spoofing es sostenido antes de declarar confirmación

#### SUSPICIOUS → NOMINAL
- Condición: `score < GSPF_TH_LOW × 0.5 = 0.25` (umbral reducido)
- Propósito: Histéresis — requiere que el score caiga bastante por debajo del umbral de entrada para volver a NOMINAL. Sin esto, el detector oscilaría cuando el score está justo en el umbral.

#### CONFIRMED → NOMINAL
- Condición: `score < GSPF_TH_LOW × 0.3 = 0.15` durante **20 ciclos** (2 s)
- Propósito: La recuperación de CONFIRMED requiere que el score sea muy bajo durante al menos 2 segundos. Esto da tiempo al CUSUM para decaer después de quitar el spoofing, y evita que un pico momentáneo de señal "limpia" cause una recuperación falsa.

### Mensajes GCS

El detector genera mensajes hacia la Ground Control Station (GCS) con diferentes severidades:

```
SUSPICIOUS: MAV_SEVERITY_WARNING   "GSPF: SUSPICIOUS score=X.XX"
CONFIRMED:  MAV_SEVERITY_CRITICAL  "GSPF: CONFIRMED SPOOFING score=X.XX"
```

Los mensajes están throttled para no saturar el canal de telemetría:
- SUSPICIOUS: máximo 1 mensaje cada 5 segundos
- CONFIRMED: máximo 1 mensaje cada 2 segundos

### Log de debug

Para diagnóstico, el detector emite periódicamente:

```
GSPF_F1: res=X.XX sAcc=X.XX nu=X.XX cusum=X.X f1=X.XX score=X.XX
GSPF_ALL: f1=X.XX f2=X.XX f3=X.XX f4=X.XX cs0=X.X cs1=X.X cs2=X.X cs3=X.X state=X
```

- `res` = módulo del residual de velocidad (m/s)
- `sAcc` = precisión de velocidad GPS reportada
- `nu` = residual normalizado (adimensional)
- `cusum` = acumulador CUSUM
- `f1..f4` = scores individuales de cada feature [0, 2]
- `cs0..cs3` = acumuladores CUSUM de cada feature
- `state` = 0=NOMINAL, 1=SUSPICIOUS, 2=CONFIRMED

---

## 11. Parámetros configurables

Todos los parámetros son accesibles desde MAVProxy o QGroundControl con el prefijo `GSPF_`:

| Parámetro | Defecto | Descripción |
|---|---|---|
| `GSPF_ENABLE` | 1 | Habilitar (1) o deshabilitar (0) el detector |
| `GSPF_INNOV_TH` | 12.0 | Umbral del CUSUM para normalizar el score. Mayor valor → más tolerante |
| `GSPF_CUSUM_K` | 1.5 | Slack del CUSUM (parámetro K). Mayor valor → menos sensible al ruido |
| `GSPF_TH_LOW` | 0.5 | Umbral de score para transición a SUSPICIOUS |
| `GSPF_TH_HIGH` | 1.0 | Umbral de score para transición a CONFIRMED |
| `GSPF_W1` | 1.0 | Peso de Feature 1 (divergencia velocidad) |
| `GSPF_W2` | 0.0 | Peso de Feature 2 (varianzas EKF) |
| `GSPF_W3` | 0.0 | Peso de Feature 3 (consistencia aceleración) |
| `GSPF_W4` | 0.0 | Peso de Feature 4 (consistencia precisión) |
| `GSPF_ACT` | 1 | Acción: 1=solo avisar, 2=acciones de seguridad adicionales |
| `GSPF_LOG_LVL` | 1 | Nivel de log (actualmente no usado, debug siempre activo) |

### Razonamiento detrás de los parámetros actuales

**`GSPF_CUSUM_K = 1.5`**: Con sigma_min = 1.0 m/s, el detector solo acumula evidencia cuando el residual supera 1.5 m/s. Residuales menores (ruido GPS normal, latencia del EKF durante maniobras) son ignorados. Este valor fue el resultado de un proceso de tuning iterativo (ver sección 13).

**`GSPF_INNOV_TH = 12.0`**: El CUSUM acumulará hasta ~12 unidades antes de que el score llegue a 1.0. Con un spoofing de 5 m/s (nu=5) y K=1.5, cada ciclo suma 3.5 unidades al CUSUM. En ~3-4 ciclos el score alcanza 1.0.

**`sigma_min = 1.0`** (hardcoded): En SITL, el GPS reporta `sAcc = 0` (precisión perfecta del simulador). Con sAcc=0, incluso residuales de 0.01 m/s causarían nu=0.1/0.0=∞ si no hubiera un mínimo. El límite de 1.0 m/s hace que el detector solo responda a residuales mayores de 1.5 m/s (K=1.5), que son claramente anómalos.

---

## 12. Archivos modificados

### Archivos nuevos creados

#### `libraries/AP_GpsSpoofDetect/AP_GpsSpoofDetect.h`
Declaración de la clase con:
- Enumeración de estados (`NOMINAL`, `SUSPICIOUS`, `CONFIRMED`)
- Parámetros configurables (`AP_Param`)
- Variables internas (acumuladores CUSUM, scores, contadores de histéresis)
- Métodos públicos de acceso y de test

#### `libraries/AP_GpsSpoofDetect/AP_GpsSpoofDetect.cpp`
Implementación completa del detector con:
- 4 funciones de cálculo de features
- Lógica de fusión y máquina de estados
- Emisión de avisos y logs

#### `libraries/AP_GpsSpoofDetect/AP_GpsSpoofDetect_config.h`
Flag de compilación condicional: `#define AP_GPSPOOFDETECT_ENABLED 1`

#### `libraries/AP_GpsSpoofDetect/LogStructure.h`
Definición de estructuras para logging Dataflash (pendiente de uso).

### Archivos de ArduCopter modificados

#### `ArduCopter/Copter.h`
Añadido el include de la librería y la instancia del detector:

```cpp
#include <AP_GpsSpoofDetect/AP_GpsSpoofDetect.h>
// ...
AP_GpsSpoofDetect gps_spoof_detect;
```

#### `ArduCopter/Copter.cpp`
Registrado en el planificador de tareas a 10 Hz:

```cpp
SCHED_TASK_CLASS(AP_GpsSpoofDetect, &copter.gps_spoof_detect, update, 10, 100, 91),
```

Esto hace que `AP_GpsSpoofDetect::update()` se llame automáticamente 10 veces por segundo, sincronizado con el ciclo GPS.

#### `ArduCopter/wscript`
Añadida la dependencia en el sistema de build:

```python
libraries += ['AP_GpsSpoofDetect']
```

### Archivos de test modificados

#### `Tools/autotest/arducopter.py`
Añadido el test automatizado `GPSSpoofGradualLoiter`:

```python
def GPSSpoofGradualLoiter(self, timeout=120):
    """Test GPS spoofing detection during loiter flight"""
```

El test:
1. Despega y vuela hacia un waypoint
2. Entra en modo LOITER
3. Inyecta errores de velocidad GPS (`SIM_GPS1_VERR_X=1.5`, `SIM_GPS1_VERR_Y=0.75`)
4. Verifica que el detector alcanza estado SUSPICIOUS en <15s
5. Verifica que el detector alcanza estado CONFIRMED en <60s
6. Quita el spoofing y verifica recuperación a NOMINAL

---

## 13. Proceso de validación y tuning

El detector pasó por múltiples iteraciones de ajuste para alcanzar el comportamiento deseado.

### Problema inicial: CUSUM demasiado sensible

**Síntoma**: Score = 0.70 en hover sin ningún spoofing, durante movimiento del vehículo.

**Causa raíz**: La normalización original usaba `sigma = max(sAcc, 0.1)`. Con `sAcc = 0` en SITL, sigma = 0.1, y residuales normales de 0.01-0.05 m/s producían nu = 0.1-0.5. Con CUSUM_K demasiado bajo (0.5-0.8), el acumulador crecía con cualquier pequeña variación.

**Investigación**: Se añadió logging de debug que reveló los valores internos:
```
GSPF_F1: res=0.02 sAcc=0.00 nu=0.20 cusum=0.0  ← correcto
...
GSPF_F1: res=0.02 sAcc=0.00 nu=0.23 cusum=22.5  ← falsa alarma
```

**Solución aplicada**: Cambiar `sigma_min` de 0.1 a 1.0. Esto hace que residuales menores de 1.5 m/s (K×sigma_min) sean completamente ignorados.

### Problema 2: Score máximo de 0.70 en lugar de 1.0

**Síntoma**: Con spoofing severo (5.5 m/s), el score nunca superaba 0.70.

**Causa raíz**: Con los pesos originales (W1=0.35, W2=0.30, W3=0.20, W4=0.15), el score máximo con solo F1 activo es:
```
score_max = W1 × f1_max / W_total = 0.35 × 2.0 / 1.0 = 0.70
```

**Solución**: Establecer W1=1.0 y W2=W3=W4=0, dando al score el rango completo [0, 2.0]. Esto también deshabilita los features que no estaban correctamente calibrados para SITL.

### Problema 3: CUSUM crece sin límite con spoofing prolongado

**Síntoma**: Tras 2-3 minutos de spoofing, el CUSUM alcanzaba valores de 300-500+. Al quitar el spoofing, el score tardaba varios minutos en bajar a NOMINAL.

**Solución**: Limitar el CUSUM a un máximo de 500:
```cpp
_cusum_pos[0] = fminf(_cusum_pos[0], 500.0f);
```

### Problema 4: Estado CONFIRMED se mantiene con score = 0

**Síntoma**: Tras quitar el spoofing, el estado seguía en CONFIRMED durante 10 segundos incluso con score = 0.00.

**Causa raíz**: La condición de recuperación de CONFIRMED requería 100 ciclos consecutivos (10 s) con score < 0.15.

**Solución**: Reducir de 100 a 20 ciclos (2 segundos), que es tiempo suficiente para que el CUSUM decaiga definitivamente.

---

## 14. Cómo probar en SITL

SITL (Software-in-the-Loop) es la herramienta de simulación de ArduPilot que ejecuta el código del autopiloto en el ordenador, conectado a un simulador de física.

### Compilar y lanzar el SITL

```bash
# Configurar para SITL
./waf configure --board=sitl

# Compilar
./waf copter

# Lanzar SITL con consola y mapa
cd ArduCopter
../Tools/autotest/sim_vehicle.py --map --console
```

### Secuencia de prueba manual

Una vez que el SITL esté corriendo y el sistema tenga GPS fix (visible en la consola), ejecutar en el prompt de MAVProxy:

```bash
# Armar y despegar
> arm throttle
> takeoff 15

# Entrar en modo de mantener posición
> mode LOITER

# Esperar 10 segundos para estabilización
# Los logs deben mostrar: score=0.00, state=0 (NOMINAL)

# Inyectar spoofing
> param set SIM_GPS1_VERR_X 5.5   # error Norte en m/s
> param set SIM_GPS1_VERR_Y 5.5   # error Este en m/s

# Observar durante 5-10 segundos:
# GSPF: CONFIRMED SPOOFING score=2.00
# EKF3 lane switch (el EKF detecta inconsistencia)

# Quitar spoofing
> param set SIM_GPS1_VERR_X 0.0
> param set SIM_GPS1_VERR_Y 0.0

# Después de ~2 segundos:
# score vuelve a 0.00, state vuelve a NOMINAL
```

### Leer los logs de debug

Para ver los valores internos del detector:

```bash
# En MAVProxy, filtrar mensajes del detector
tail -f /tmp/mavproxy.log | grep GSPF
```

### Ejecutar el test automatizado

```bash
cd /path/to/ardupilot
Tools/autotest/autotest.py build.Copter test.Copter.GPSSpoofGradualLoiter
```

---

## 15. Limitaciones conocidas y trabajo futuro

### Limitaciones actuales

1. **Solo F1 está activo**: Las features F2 (varianzas EKF), F3 (consistencia aceleración) y F4 (consistencia precisión) tienen peso 0. Esto reduce la robustez ante ataques que no causen divergencia de velocidad directa.

2. **Sin logging Dataflash**: Los avisos actuales van solo a GCS. Para análisis post-vuelo, sería necesario implementar logging binario en el fichero `.bin` del datalogger.

3. **Sin acción de contingencia**: El parámetro `GSPF_ACT = 2` está definido pero no implementado. En el futuro podría activar modo RTL (Return to Launch) o aterrizar al detectar spoofing confirmado.

4. **sigma_min hardcoded**: El valor de 1.0 m/s para la normalización mínima está hardcoded en el código. Debería ser un parámetro ajustable para adaptarse a distintos receptores GPS que reporten sAcc de formas diferentes.

5. **Solo un GPS**: El detector compara el GPS primario con el EKF. Con múltiples receptores GPS, se podría implementar comparación cruzada entre receptores.

### Trabajo futuro

- Calibrar y habilitar F2, F3, F4 en hardware real
- Implementar logging Dataflash
- Implementar acción de contingencia en estado CONFIRMED
- Agregar soporte para comparación entre múltiples receptores GPS
- Evaluar rendimiento con ataques de spoofing de distintas velocidades y perfiles
- Añadir soporte para notificación MAVLink a GCS con mensaje específico de spoofing
