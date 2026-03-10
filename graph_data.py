# import matplotlib.pyplot as plt

# time_us = []
# velocity = []

# with open("velocity_data.txt", "r") as f:
#     for line in f:
#         line = line.strip()

#         if not line:
#             continue
#         if line.startswith("Time"):
#             continue
#         if "," not in line:
#             continue

#         t, v = line.split(",")
#         time_us.append(int(t))
#         velocity.append(float(v))

# # Convert to seconds
# time_s = [(t - time_us[0]) / 1e6 for t in time_us]

# plt.plot(time_s, velocity)
# plt.xlabel("Time (s)")
# plt.ylabel("Velocity (ticks/s)")
# plt.title("Motor Velocity vs Time")
# plt.grid(True)
# plt.show()
#code "C:\Users\nikis\OneDrive - Cal Poly\405\Lab_0x03_Starter"
# import matplotlib.pyplot as plt

# #FILENAME = "velocity_data_4fun.txt"   # PuTTY log file
# FILENAME = "velocity_data.txt"   # PuTTY log file

# runs = []            # list of runs; each run = list of (time_us, velocity)
# current_run = []
# recording = False

# with open(FILENAME, "r", encoding="utf-8", errors="ignore") as f:
#     for line in f:
#         line = line.strip()

#         # Start of a data block
#         if line.startswith("Time, Velocity"):
#             current_run = []
#             recording = True
#             continue

#         # End of a data block
#         if recording and line.startswith("--------------------"):
#             if current_run:
#                 runs.append(current_run)
#             recording = False
#             continue

#         if not recording:
#             continue

#         # Handle lines like: "30595,1200.0,"
#         parts = [p.strip() for p in line.split(",") if p.strip() != ""]
#         if len(parts) < 2:
#             continue

#         try:
#             t = int(parts[0])
#             v = float(parts[1])
#         except ValueError:
#             continue

#         current_run.append((t, v))

# # Catch last run if file doesn't end with dashes
# if recording and current_run:
#     runs.append(current_run)

# if not runs:
#     raise RuntimeError("No valid data found. Check PuTTY logging output.")

# # -------- PLOTTING --------
# plt.figure()

# for i, run in enumerate(runs):
#     t0 = run[0][0]
#     time_s = [(t - t0) / 1e6 for t, _ in run]
#     velocity = [v for _, v in run]

#     plt.plot(time_s, velocity, label=f"Run {i + 1}")

# plt.xlabel("Time (s)")
# plt.ylabel("Velocity (ticks/s)")
# plt.title("Velocity vs Time")
# plt.grid(True)
# plt.legend()
# plt.tight_layout()
# plt.show()


import matplotlib.pyplot as plt

#FILENAME = "velocity_data_4fun.txt"   # PuTTY log file
FILENAME = "velocity_data.txt"        # PuTTY log file

runs = []            # list of runs; each run = list of (time_us, velocity)
current_run = []
recording = False

with open(FILENAME, "r", encoding="utf-8", errors="ignore") as f:
    for line in f:
        line = line.strip()

        # Start of a data block
        if line.startswith("Time, Velocity"):
            current_run = []
            recording = True
            continue

        # End of a data block
        if recording and line.startswith("--------------------"):
            if current_run:
                runs.append(current_run)
            recording = False
            continue

        if not recording:
            continue

        # Handle lines like: "30595,1200.0,"
        parts = [p.strip() for p in line.split(",") if p.strip() != ""]
        if len(parts) < 2:
            continue

        try:
            t = int(parts[0])
            v = float(parts[1])
        except ValueError:
            continue

        current_run.append((t, v))

# Catch last run if file doesn't end with dashes
if recording and current_run:
    runs.append(current_run)

if not runs:
    raise RuntimeError("No valid data found. Check PuTTY logging output.")

# -------- PLOTTING --------
plt.figure()

run_names = ["Left", "Right"]

for i, run in enumerate(runs):
    t0 = run[0][0]
    time_s = [(t - t0) / 1e6 for t, _ in run]
    velocity = [v for _, v in run]

    label = run_names[i] if i < len(run_names) else f"Run {i+1}"

    plt.plot(time_s, velocity, label=label)

plt.xlabel("Time (s)")
plt.ylabel("Velocity (ticks/s)")
plt.title("Velocity vs Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()