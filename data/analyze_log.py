"""
Анализ телеметрии термостата.

Читает CSV-лог (полученный через Serial Monitor -> Save as CSV,
или через pyserial напрямую), строит графики:
  1. Plate_Temp / Outdoor_Temp / Setpoint во времени
  2. PWM во времени с разбивкой по состояниям (BOOT/HEATING/STABILIZED/ERROR)
  3. Сравнение теоретического предсказания установившегося PWM
     (по формуле PWM = a + Kloss * (setpoint - outdoor)) с фактически
     наблюдаемым PWM в стабильном режиме — для отчёта "теория vs эксперимент"

Использование:
    python analyze_log.py telemetry.csv
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt

def main(csv_path: str):
    df = pd.read_csv(csv_path)
    df["time_s"] = (df["time_ms"] - df["time_ms"].iloc[0]) / 1000.0

    fig, axes = plt.subplots(4, 1, figsize=(10, 13), sharex=False)

    # --- График 1: температуры ---
    axes[0].plot(df["time_s"], df["plate_temp"], label="Пластина (факт)")
    axes[0].plot(df["time_s"], df["outdoor_temp"], label="Улица (факт)")
    axes[0].plot(df["time_s"], df["setpoint"], "--", label="Setpoint")
    axes[0].set_ylabel("Температура, °C")
    axes[0].legend()
    axes[0].set_title("Температуры во времени")

    # --- График 2: ШИМ и состояния ---
    axes[1].plot(df["time_s"], df["pwm"], color="orange", label="PWM")
    axes[1].set_ylabel("PWM (0-255)")
    axes[1].legend()
    axes[1].set_title("Управляющий сигнал")

    # --- График 3: теория vs эксперимент (только в STABILIZED) ---
    stable = df[df["state"] == "STABILIZED"].copy()
    if not stable.empty:
        stable["delta_t"] = stable["setpoint"] - stable["outdoor_temp"]
        # Теоретическое предсказание по последней оценке Kloss в конце лога
        kloss_final = stable["est_kloss"].iloc[-1]
        stable["pwm_theory"] = kloss_final * stable["delta_t"]

        axes[2].scatter(stable["delta_t"], stable["pwm"], s=8,
                         label="Эксперимент (факт. PWM)")
        axes[2].plot(stable["delta_t"].sort_values(),
                      kloss_final * stable["delta_t"].sort_values(),
                      color="red", label=f"Теория (Kloss={kloss_final:.2f})")
        axes[2].set_xlabel("ΔT = Setpoint − Outdoor, °C")
        axes[2].set_ylabel("PWM в установившемся режиме")
        axes[2].legend()
        axes[2].set_title("Сравнение теоретической модели теплопотерь с экспериментом")
    else:
        axes[2].text(0.5, 0.5, "Нет данных в состоянии STABILIZED",
                     ha="center", va="center")

    plt.tight_layout()
    out_path = csv_path.replace(".csv", "_analysis.png")
    plt.savefig(out_path, dpi=150)
    print(f"График сохранён: {out_path}")

    # Численная сводка для отчёта
    if not stable.empty:
        residuals = stable["pwm"] - stable["pwm_theory"]
        print(f"Оценка Kloss (финальная): {kloss_final:.3f}")
        print(f"Средняя абсолютная ошибка теория/эксперимент: {residuals.abs().mean():.2f} PWM-ед.")
        print(f"RMSE: {(residuals**2).mean()**0.5:.2f} PWM-ед.")

    # --- Дополнительно: если в логе есть колонки Kp/Ki/Kd (после автонастройки) ---
    if {"Kp", "Ki", "Kd"}.issubset(df.columns):
        axes[3].plot(df["time_s"], df["Kp"], label="Kp")
        axes[3].plot(df["time_s"], df["Ki"], label="Ki")
        axes[3].plot(df["time_s"], df["Kd"], label="Kd")
        axes[3].set_xlabel("Время, с")
        axes[3].set_ylabel("Коэффициенты PID")
        axes[3].legend()
        axes[3].set_title("Коэффициенты PID (скачок = момент завершения автонастройки)")

        # Отмечаем момент перехода AUTOTUNE -> HEATING на всех графиках
        autotune_mask = df["state"] == "AUTOTUNE"
        if autotune_mask.any():
            autotune_end_time = df.loc[autotune_mask, "time_s"].max()
            for ax in axes:
                ax.axvline(autotune_end_time, color="gray", linestyle=":",
                           label="Конец автонастройки" if ax is axes[0] else None)
            axes[0].legend()
            print(f"Автонастройка завершена на {autotune_end_time:.1f} с; "
                  f"итоговые Kp={df['Kp'].iloc[-1]:.2f}, "
                  f"Ki={df['Ki'].iloc[-1]:.2f}, Kd={df['Kd'].iloc[-1]:.2f}")

        plt.tight_layout()
        plt.savefig(out_path, dpi=150)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Использование: python analyze_log.py <путь_к_csv>")
        sys.exit(1)
    main(sys.argv[1])
