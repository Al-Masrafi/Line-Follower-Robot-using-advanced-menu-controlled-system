from matplotlib import pyplot as plt
import numpy as np

def import_data(suffix):
    position = []
    time = []
    filename = "data" + str(suffix) + ".txt"
    path = r"C:\Users\Neo\Documents\GitHub\KEX\ZN" + filename
    with open(path, "r") as file:
        for row in file:
            row = row.split()
            position.append(row[0])
            time.append(row[1])
    return position, time

def treat_data(all_pos, all_time):
    a = 0.1
    for i in range(len(all_time)):
        all_time[i] = float(all_time[i]) / 1000
        all_pos[i] = float(all_pos[i])
        if i + 1 == len(all_time):
            break
        all_pos[i + 1] = a * float(all_pos[i + 1]) + (1 - a) * all_pos[i]
    return all_time, all_pos

def plot_data(all_time, all_pos, i):
    plt.plot(all_time, all_pos)
    plt.xlabel("Time [sec]")
    plt.ylabel("Positional value")
    plt.grid()
    plt.axis([0, 4, 0, 6000])
    plt.title("Ziegler-Nichols test " + str(i + 1))
    plt.savefig("ZN" + str(i + 1))

def fit_sine(time, pos):
    start_time = time[0]
    end_time = time[-1]
    crosses = 0
    state = pos[0] > 3500
    for i in range(len(time)):
        if pos[i] > 3500 and state is False:
            state = True
            crosses += 1
        elif pos[i] < 3500 and state is True:
            state = False
            crosses += 1
    return 2 / (crosses / (end_time - start_time))

def average(lst):
    return sum(lst) / len(lst)

def rise_time(time, pos):
    maxvalue = max(pos)
    for i in range(len(pos)):
        if pos[i] >= 0.1 * maxvalue:
            low_time = time[i]
            break
    for i in range(len(pos)):
        if pos[i] >= 0.9 * maxvalue:
            high_time = time[i]
            break
    return high_time - low_time

def overshoot(time, pos):
    maxvalue = max(pos)
    last_fifty = len(pos) - 50
    avg = average(pos[last_fifty:])
    return 100 * (maxvalue - avg) / avg, avg

def settling_time(time, pos):
    last_fifty = len(pos) - 50
    avg = average(pos[last_fifty:])
    top_value = 1.05 * avg
    low_value = 0.95 * avg
    inside = False
    for i in range(len(pos)):
        if pos[i] <= top_value and pos[i] >= low_value:
            if not inside:
                settling = time[i]
                inside = True
        else:
            inside = False
    return settling if inside else max(time)

def main():
    all_pos = []
    all_time = []
    try:
        for i in range(10):
            pos, tim = import_data(i + 1)
            all_pos.append(pos)
            all_time.append(tim)
    except Exception as e:
        print(e)

    oversh = []
    rise = []
    settling = []
    avg = []
    for i in range(len(all_pos)):
        all_time[i], all_pos[i] = treat_data(all_pos[i], all_time[i])
        over, av = overshoot(all_time[i], all_pos[i])
        oversh.append(over)
        avg.append(av)
        rise.append(rise_time(all_time[i], all_pos[i]))
        settling.append(settling_time(all_time[i], all_pos[i]))
        plot_data(all_time[i], all_pos[i], i)

    print("Rise time:", round(np.average(rise), 4), "±", round(np.std(rise), 4))
    print("Overshoot:", round(np.average(oversh), 2), "±", round(np.std(oversh), 2))
    print("Settling time:", round(np.average(settling), 2), "±", round(np.std(settling), 2))
    print("Settling value:", round(np.average(avg), 2), "±", round(np.std(avg), 2))
    plt.show()

if __name__ == "__main__":
    main()
