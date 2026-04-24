import numpy as np
import matplotlib.pyplot as plt


# titles = ("Position 1", "Position 2", "Position 3")
labels = ("Bana 1", "Bana 2", "Bana 3")
data_025 = {
    "Uttömmande" : (97.89, 97.71,97.64),
    "Enkel modell" : (99.01,98.52,98.67),
    "Avancerad modell" : (99.65,99.35,99.05),
}
data_050 = {
    "Uttömmande" : (99.55, 86.27,96.39),
    "Enkel modell" : (87.96,59.25,84.13),
    "Avancerad modell" : (97.29,84.11,95.64),
}

labels_time = ("Importtid", "Transformeringstid", "Söktid", "Total tid")
# data_time = {
#     "Uttömmande":       (20.29, 60.61, 112.88),
#     "Enkel modell":     (19.51, 52.61, 1.43),
#     "Avancerad modell": (19.64, 52.35, 1.30),
# }
data_time = {
    "Uttömmande":       (20.3, 60.6, 112.9),
    "Enkel modell":     (19.5, 52.6, 1.4),
    "Avancerad modell": (19.6, 52.4, 1.30),
}

def plot_acc(data, spd, y_limit):
    fig, ax = plt.subplots(layout="constrained", figsize = (8,5))
    # fig.set_figheight(10)

    x = np.arange(3)
    width = 0.2
    multiplier = 0
    spacing = 0.04
    # x = np.multiply(x,1.1)

    for attribute, measurement in data.items():
        offset = (width+spacing) * multiplier
        rects = ax.bar(x+offset, measurement, width, label=attribute)
        ax.bar_label(rects, padding=3,rotation = 45)
        multiplier += 1

    ax.set_xticks(x + (width+spacing), labels)
    ax.legend(title = "Söktyp",loc = "center right")
    ax.set_title(f"v = {spd} m/s")
    ax.set_ylabel("Träffsäkerhet[%]", weight = 'bold')
    ax.set_xlabel("Testbana", weight = 'bold')
    if spd == "0.50":
        ax.set_ylim(y_limit,105)
    else:
        ax.set_ylim(y_limit, 101)
    ax.set_xlim(-0.3,3.6)

    plt.draw()

def plot_time(data):
    fig, ax = plt.subplots(layout="constrained", figsize = (8,5))
    # fig.set_figheight(10)
    for h in range(25, 201, 25):
        ax.axhline(y=h, color="gray", linewidth=1, zorder=0.1)

    x = np.arange(4)
    width = 0.2
    multiplier = 0
    spacing = 0.04
    # x = np.multiply(x,1.1)

    for attribute, measurement in data.items():
        offset = (width+spacing) * multiplier
        bar = [elem for elem in measurement]
        bar.append(sum(measurement))
        rects = ax.bar(x+offset, bar, width, label=attribute)
        #ax.bar_label(rects, padding=3,rotation = 45)
        multiplier += 1

    ax.set_xticks(x + (width+spacing), labels_time)
    ax.legend(title = "Söktyp",loc = "center right")
    ax.set_title(f"Tidsprofilering")
    ax.set_ylabel("Tidsåtgång [ms]", weight = 'bold')
    # ax.set_xlabel("Testbana", weight = 'bold')
    ax.set_ylim(0, 200)
    ax.set_xlim(-0.3,4.8)

    plt.draw()



# plot_acc(data_025, "0.25", 90)
# plot_acc(data_050, "0.50", 50)
plot_time(data_time)
plt.show()