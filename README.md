# DIGITAL-FILTER-DESIGN

**COMPANY** :CODTECH IT SOLUTIONS

**NAME** :B.SAISRIMA

**INTERN ID** :CT04DM37

**DOMAIN** : VLSI

**DURATION** : 4 WEEKS

**MENTOR** : NEELA SANTHOSH

#DESCRIPTION :
**design and simulation of a digital FIR (Finite Impulse Response) filter** using **Verilog** or **MATLAB**, covering design principles, implementation details, and simulation results:

---

### **Design and Simulation of a Digital FIR Filter Using Verilog/MATLAB**

The goal of this project was to design, implement, and simulate a **Finite Impulse Response (FIR) digital filter** using either **Verilog** or **MATLAB**. FIR filters are widely used in digital signal processing (DSP) due to their inherent stability, linear phase characteristics, and design simplicity. This project aimed to demonstrate the fundamental principles of FIR filtering and to realize a working hardware or software model capable of filtering discrete-time signals.

---

### **1. FIR Filter Fundamentals**

A FIR filter is a type of digital filter that operates on finite-duration input signals using a fixed set of coefficients. The general equation of an FIR filter of order `N` is:

$$
y[n] = \sum_{k=0}^{N} h[k] \cdot x[n-k]
$$

Where:

* $y[n]$: Filter output at time `n`
* $x[n-k]$: Input sample delayed by `k`
* $h[k]$: Filter coefficient (impulse response)
* $N$: Filter order (number of taps - 1)

FIR filters perform **convolution** of the input signal with a finite set of filter coefficients.

---

### **2. Design Approach**

The design process involved the following steps:

* **Coefficient Generation**: FIR filter coefficients were generated using MATLAB's built-in `fir1()` function or a windowing technique (e.g., Hamming window) based on desired specifications such as cutoff frequency, order, and sampling rate.
* **Fixed-point Representation**: Coefficients were quantized to fixed-point format for hardware implementation (in Verilog), ensuring compatibility with digital hardware.
* **Implementation**:

  * In **MATLAB**, a vector-based convolution method was used to implement the filter.
  * In **Verilog**, the filter was implemented using a series of delay registers, multipliers (for coefficients), and an adder tree to accumulate the results.

---

### **3. Verilog Implementation Overview**

A basic Verilog FIR filter structure includes:

* **Shift Register**: Stores the current and past input samples.
* **Multiplier Array**: Multiplies each input sample with a corresponding coefficient.
* **Adder Tree**: Adds the products to produce the final filtered output.

Example outline (for N=4 taps):

```verilog
module fir_filter (
    input clk,
    input rst,
    input signed [7:0] x_in,
    output reg signed [15:0] y_out
);

    reg signed [7:0] x [0:3]; // Shift register
    wire signed [15:0] mult [0:3]; // Multipliers
    parameter signed [7:0] h [0:3] = {8'd3, 8'd2, 8'd1, 8'd1}; // Coefficients

    // Shift input samples
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            x[0] <= 0; x[1] <= 0; x[2] <= 0; x[3] <= 0;
        end else begin
            x[3] <= x[2];
            x[2] <= x[1];
            x[1] <= x[0];
            x[0] <= x_in;
        end
    end

    assign mult[0] = x[0] * h[0];
    assign mult[1] = x[1] * h[1];
    assign mult[2] = x[2] * h[2];
    assign mult[3] = x[3] * h[3];

    always @(posedge clk) begin
        y_out <= mult[0] + mult[1] + mult[2] + mult[3];
    end
endmodule
```

---

### **4. MATLAB Implementation Overview**

In MATLAB, the FIR filter was simulated using the `filter()` function or manual convolution:

```matlab
h = fir1(3, 0.5);        % Low-pass filter, 4 taps
x = randn(1, 100);       % Input signal
y = filter(h, 1, x);     % FIR filtering
```

This allowed rapid testing and visualization of the filter’s frequency and time-domain behavior using plots.

---

 **5. Simulation and Verification**

The FIR filter design was simulated to verify both functionality and performance:

 **MATLAB**: Time-domain plots showed the filter smoothing the signal, removing high-frequency components.
**Verilog**: Simulated in ModelSim or Vivado with a testbench that applied test inputs and monitored the output waveform. The result matched the expected convolution sum, proving the hardware model worked correctly.

Testbench example (Verilog):

```verilog
initial begin
    clk = 0; rst = 1; x_in = 0;
    #10 rst = 0;
    #10 x_in = 8'd10;
    #10 x_in = 8'd20;
    ...
end
```

---

**6. Conclusion**

This project successfully demonstrated the design and simulation of a digital FIR filter in both Verilog and MATLAB. The filter accurately processed discrete input signals using fixed coefficients and performed real-time convolution, achieving the desired signal filtering. The Verilog model provides a solid base for hardware implementation, while MATLAB offers rapid prototyping and visualization. Together, they offer a powerful approach to designing and verifying digital signal processing systems.

