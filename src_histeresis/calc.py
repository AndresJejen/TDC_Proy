import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("/Users/germanandresjejencortes/UNAL/CONTROL_LAB/FINAL_PROY/ESP32_CODE/src/histeresis_result.csv")

plt.figure(figsize=(40, 10))

plt.plot(data.Time, data.y)
plt.plot(data.Time, data.u)
plt.show()


