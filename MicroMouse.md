# Micromouse

### **Understanding PID Parameters for Your Micromouse**

Since your micromouse has **two PID controllers**, one for **position control** and one for **synchronization**, I'll break down the roles of **Kp, Ki, and Kd** for both.

---

## **1. Position PID (leftPID & rightPID)**

Controls how far each motor should move based on **encoder counts**.

| **Parameter** | **Purpose** | **Tuning Effect** |
| --- | --- | --- |
| **Kp (Proportional Gain)** | Reacts to how far the encoder count is from the target distance. | 🚀 **Too high** → Motor moves too fast, overshoots.  🐢 **Too low** → Motor moves too slow. |
| **Ki (Integral Gain)** | Compensates for small, consistent errors over time (like friction or battery voltage drop). | 🔄 **Too high** → Motor keeps accelerating, overshoots badly.  🔄 **Too low** → Motor never reaches the exact target. |
| **Kd (Derivative Gain)** | Dampens sudden changes in speed (helps smooth braking and stopping). | 🔥 **Too high** → Motor shakes, unstable.  🏁 **Too low** → Motor stops too slowly, overshoots. |

**💡 How to Tune (Position PID)**

1. Start with **only Kp**, increase until the motor moves well but slightly overshoots.
2. Increase **Kd** to reduce overshooting and oscillations.
3. Increase **Ki** to ensure the motor reaches the exact target, but avoid too much overshoot.

---

## **2. Synchronization PID (syncPID)**

Ensures **both wheels stay at the same encoder count**, preventing drift.

| **Parameter** | **Purpose** | **Tuning Effect** |
| --- | --- | --- |
| **Kp_sync** | Corrects immediate differences in encoder counts. | 🚀 **Too high** → Bot keeps making aggressive corrections, jittery.  🐢 **Too low** → One wheel drifts, not staying synchronized. |
| **Ki_sync** | Fixes **small long-term drift** between the two wheels. | 🔄 **Too high** → Bot starts making slow but unnecessary corrections.  🔄 **Too low** → Small drift accumulates over time. |
| **Kd_sync** | Smooths out sudden synchronization adjustments. | 🔥 **Too high** → Motors keep making small jerky movements.  🏁 **Too low** → Bot takes too long to correct drift. |

**💡 How to Tune (Synchronization PID)**

1. Start with **only Kp_sync**, increase until both wheels stay in sync without too many corrections.
2. Increase **Kd_sync** if the bot shakes or jitters when correcting drift.
3. Adjust **Ki_sync** if one wheel slowly pulls ahead over time.

---

## **Summary of Tuning Strategy**

- **Position PID (Kp, Ki, Kd)** ensures the bot moves **exactly 26 cm and stops precisely**.
- **Sync PID (Kp_sync, Ki_sync, Kd_sync)** ensures **both wheels hit the same encoder count at the same time**.

### **Recommended Tuning Order**

1. **Set Ki and Kd to 0** for both PIDs.
2. Increase **Kp** (position) until the bot moves fast but overshoots slightly.
3. Increase **Kd** (position) to reduce overshoot.
4. Add **Ki** (position) to ensure it reaches exactly 26 cm.
5. Set **Kp_sync** and tune it until both wheels stay synchronized.
6. Add **Kd_sync** to smooth out any aggressive corrections.
7. If one wheel slowly drifts over time, add **Ki_sync**.

---

### **Final Notes**

✔ If your bot **overshoots too much**, increase **Kd** and lower **Kp**.

✔ If your bot **stops before 26 cm**, increase **Ki** slightly.

✔ If one wheel moves faster than the other, tune **syncPID** separately.

Would you like an **auto-tuning approach** or help with **real-time debugging**? 🚀

# POA

### **Sequential Plan to Tune PID for Your Micromouse**

To properly tune both the **position PID** (for reaching 26 cm) and **sync PID** (for keeping both wheels in sync), follow these steps in order.

---

## **Phase 1: Setup for PID Tuning**

1️⃣ **Disable Sync PID Initially**

- Comment out `syncPID.Compute();` in `loop()`.
- This ensures that we first tune position control without interference.

2️⃣ **Set Initial PID Values**

- Set **Ki and Kd to 0** and start only with **Kp** for position control.

```cpp
double Kp = 5.0, Ki = 0.0, Kd = 0.0;  // Start with Ki and Kd as 0

```

---

## **Phase 2: Tune Position PID (Reaching 26 cm Exactly)**

🚀 **Goal:** Get the bot to **move forward and stop at exactly 26 cm**.

3️⃣ **Increase Kp Until the Bot Moves Well**

- Gradually **increase Kp** until the bot moves **reasonably fast** but overshoots slightly.
- If it **doesn’t move**, keep increasing Kp.
- If it **oscillates wildly**, reduce Kp slightly.

4️⃣ **Increase Kd to Reduce Overshoot**

- Once the bot moves but **overshoots**, start increasing **Kd**.
- If Kd is too high, the bot will **shake/vibrate** when stopping—reduce it slightly.

5️⃣ **Increase Ki to Remove Small Position Errors**

- If the bot **stops before 26 cm**, increase **Ki** to remove small steady-state errors.
- If Ki is **too high**, the bot might overshoot and struggle to settle.

✅ **At this point, the bot should stop exactly at 26 cm.**

---

## **Phase 3: Tune Sync PID (Keeping Both Wheels in Sync)**

🚀 **Goal:** Ensure **both wheels reach the same encoder count at the same time** (no drift).

6️⃣ **Enable Sync PID (`syncPID.Compute();`)**

- Uncomment `syncPID.Compute();` in `loop()`.
- Set **Ki_sync and Kd_sync to 0**, start with only **Kp_sync**.

```cpp
double Kp_sync = 2.0, Ki_sync = 0.0, Kd_sync = 0.0;

```

7️⃣ **Increase Kp_sync to Minimize Drift**

- Run the bot and **observe if one wheel moves faster** than the other.
- Gradually increase **Kp_sync** until both wheels stay in sync.
- If **too high**, the bot will jitter as it constantly corrects—reduce slightly.

8️⃣ **Increase Kd_sync to Smooth Corrections**

- If the bot **shakes when correcting drift**, increase **Kd_sync**.
- If Kd_sync is **too high**, the bot may be sluggish to adjust—reduce slightly.

9️⃣ **Increase Ki_sync to Remove Long-Term Drift**

- If one wheel **slowly drifts ahead over time**, increase **Ki_sync**.
- If Ki_sync is **too high**, the bot will make unnecessary small corrections—reduce slightly.

✅ **At this point, both wheels should move together and stop exactly at 26 cm.**

---

## **Final Testing & Adjustments**

🔟 **Test Multiple Runs**

- Run the bot **multiple times** to check consistency.
- If results **vary**, adjust **Ki** (position) or **Ki_sync** (sync) slightly.

1️⃣1️⃣ **Check for Oscillations or Instability**

- If the bot **oscillates** or vibrates, reduce **Kd** or **Kp**.
- If it **reacts too slowly**, increase **Kp** slightly.

---

## **Summary of Tuning Steps**

| **Step** | **Action** | **Expected Result** |
| --- | --- | --- |
| **1-2** | Disable sync PID, start with Kp only | Only position control is active |
| **3** | Increase Kp (position) | Bot moves but overshoots |
| **4** | Increase Kd (position) | Overshoot is reduced |
| **5** | Increase Ki (position) | Bot stops exactly at 26 cm |
| **6** | Enable sync PID | Both wheels should move together |
| **7** | Increase Kp_sync | Wheels stay synced, no drift |
| **8** | Increase Kd_sync | Smooth corrections, no jerks |
| **9** | Increase Ki_sync | No long-term drift |
| **10-11** | Test and refine | Smooth movement, no overshoot |

---

### **Key Takeaways**

- **Start with only Kp** for each PID.
- **Tackle position control first**, then sync control.
- **Tune Kd to reduce overshooting**, Ki to remove steady-state errors.
- **Sync PID prevents one wheel from moving faster than the other.**
- **Test in small steps** to avoid drastic changes.

---

🚀 **With this plan, your micromouse will reach exactly 26 cm and stay in sync!** 🐭

Would you like a simple **data logging method** to fine-tune your PID values in real-time?

### **Sequential Plan to Tune PID for Your Micromouse**

To properly tune both the **position PID** (for reaching 26 cm) and **sync PID** (for keeping both wheels in sync), follow these steps in order.

---

## **Phase 1: Setup for PID Tuning**

1️⃣ **Disable Sync PID Initially**

- Comment out `syncPID.Compute();` in `loop()`.
- This ensures that we first tune position control without interference.

2️⃣ **Set Initial PID Values**

- Set **Ki and Kd to 0** and start only with **Kp** for position control.

```cpp
double Kp = 5.0, Ki = 0.0, Kd = 0.0;  // Start with Ki and Kd as 0

```

---

## **Phase 2: Tune Position PID (Reaching 26 cm Exactly)**

🚀 **Goal:** Get the bot to **move forward and stop at exactly 26 cm**.

3️⃣ **Increase Kp Until the Bot Moves Well**

- Gradually **increase Kp** until the bot moves **reasonably fast** but overshoots slightly.
- If it **doesn’t move**, keep increasing Kp.
- If it **oscillates wildly**, reduce Kp slightly.

4️⃣ **Increase Kd to Reduce Overshoot**

- Once the bot moves but **overshoots**, start increasing **Kd**.
- If Kd is too high, the bot will **shake/vibrate** when stopping—reduce it slightly.

5️⃣ **Increase Ki to Remove Small Position Errors**

- If the bot **stops before 26 cm**, increase **Ki** to remove small steady-state errors.
- If Ki is **too high**, the bot might overshoot and struggle to settle.

✅ **At this point, the bot should stop exactly at 26 cm.**

---

## **Phase 3: Tune Sync PID (Keeping Both Wheels in Sync)**

🚀 **Goal:** Ensure **both wheels reach the same encoder count at the same time** (no drift).

6️⃣ **Enable Sync PID (`syncPID.Compute();`)**

- Uncomment `syncPID.Compute();` in `loop()`.
- Set **Ki_sync and Kd_sync to 0**, start with only **Kp_sync**.

```cpp
double Kp_sync = 2.0, Ki_sync = 0.0, Kd_sync = 0.0;

```

7️⃣ **Increase Kp_sync to Minimize Drift**

- Run the bot and **observe if one wheel moves faster** than the other.
- Gradually increase **Kp_sync** until both wheels stay in sync.
- If **too high**, the bot will jitter as it constantly corrects—reduce slightly.

8️⃣ **Increase Kd_sync to Smooth Corrections**

- If the bot **shakes when correcting drift**, increase **Kd_sync**.
- If Kd_sync is **too high**, the bot may be sluggish to adjust—reduce slightly.

9️⃣ **Increase Ki_sync to Remove Long-Term Drift**

- If one wheel **slowly drifts ahead over time**, increase **Ki_sync**.
- If Ki_sync is **too high**, the bot will make unnecessary small corrections—reduce slightly.

✅ **At this point, both wheels should move together and stop exactly at 26 cm.**

---

## **Final Testing & Adjustments**

🔟 **Test Multiple Runs**

- Run the bot **multiple times** to check consistency.
- If results **vary**, adjust **Ki** (position) or **Ki_sync** (sync) slightly.

1️⃣1️⃣ **Check for Oscillations or Instability**

- If the bot **oscillates** or vibrates, reduce **Kd** or **Kp**.
- If it **reacts too slowly**, increase **Kp** slightly.

---

## **Summary of Tuning Steps**

| **Step** | **Action** | **Expected Result** |
| --- | --- | --- |
| **1-2** | Disable sync PID, start with Kp only | Only position control is active |
| **3** | Increase Kp (position) | Bot moves but overshoots |
| **4** | Increase Kd (position) | Overshoot is reduced |
| **5** | Increase Ki (position) | Bot stops exactly at 26 cm |
| **6** | Enable sync PID | Both wheels should move together |
| **7** | Increase Kp_sync | Wheels stay synced, no drift |
| **8** | Increase Kd_sync | Smooth corrections, no jerks |
| **9** | Increase Ki_sync | No long-term drift |
| **10-11** | Test and refine | Smooth movement, no overshoot |

---

### **Key Takeaways**

- **Start with only Kp** for each PID.
- **Tackle position control first**, then sync control.
- **Tune Kd to reduce overshooting**, Ki to remove steady-state errors.
- **Sync PID prevents one wheel from moving faster than the other.**
- **Test in small steps** to avoid drastic changes.

---

🚀 **With this plan, your micromouse will reach exactly 26 cm and stay in sync!** 🐭

Would you like a simple **data logging method** to fine-tune your PID values in real-time?

# Drift conditions

### **How to Identify if the Micromouse Moves Exactly 26 cm & Detect Drift?**

Before tuning, you need a **way to measure** whether the bot reaches **exactly 26 cm** and whether it's **drifting (one wheel moving faster than the other).**

---

## **Step 1: Use Encoder Counts to Verify Distance**

Since your motor encoders count pulses per rotation, you can calculate how far the bot has traveled.

1️⃣ **Formula to Convert Encoder Counts to Distance:**

Distance=Encoder CountsCounts per cm\text{Distance} = \frac{\text{Encoder Counts}}{\text{Counts per cm}}

- Example: If **Counts per cm = 100** and encoder count reaches **2600**, the bot has moved **26 cm**.

2️⃣ **Modify Serial Output to Print Real-time Distance:**

Add this inside `loop()`:

```cpp
Serial.print("Left Distance: ");
Serial.print(leftCount / COUNTS_PER_CM);
Serial.print(" cm, Right Distance: ");
Serial.print(rightCount / COUNTS_PER_CM);
Serial.println(" cm");

```

- This will **print the actual distance traveled** by each wheel.

---

## **Step 2: Detect if the Bot is Drifting**

🚨 **Drift happens when one wheel moves faster than the other.**

- If **leftCount > rightCount**, the bot **drifts right**.
- If **rightCount > leftCount**, the bot **drifts left**.

### **Check for Drift in Serial Monitor**

Modify your `loop()` to print drift information:

```cpp
int driftError = leftCount - rightCount;  // Difference in encoder counts
Serial.print("Drift Error: ");
Serial.println(driftError);

```

- If **driftError is positive**, the left wheel is ahead.
- If **driftError is negative**, the right wheel is ahead.

---

## **Step 3: Tune Position PID First (Ignoring Drift)**

🚀 **Goal:** Get the bot to move **exactly 26 cm**, even if it drifts.

- Start with **Kp only** and check **if the encoder counts reach the target (2600 counts).**
- If the bot **stops too early**, **increase Kp**.
- If the bot **overshoots**, **increase Kd**.

Once you get **leftCount ≈ 2600 and rightCount ≈ 2600**, **move to sync tuning**.

---

## **Step 4: Tune Sync PID to Eliminate Drift**

🚀 **Goal:** Make **leftCount and rightCount equal** throughout movement.

- Enable `syncPID.Compute();` and gradually increase **Kp_sync**.
- If `Drift Error` is **reducing**, you're tuning correctly.
- If the bot **jitters**, increase **Kd_sync** to smooth corrections.

---

## **Final Testing Checklist**

✅ **Does `leftCount` and `rightCount` reach 2600?**

✅ **Is `Drift Error` close to 0 throughout the run?**

✅ **Does the bot stop precisely at 26 cm?**

---

### **🎯 Now You Can Tune PID with Confidence! 🚀**

Let me know if you need a logging method to record PID behavior for fine-tuning!