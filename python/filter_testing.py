import numpy as np
import matplotlib.pyplot as plt

############################ Low Pass Filter ######################## 
# Calculate the alpha value based on cutoff frequency
def calculate_alpha(cutoff_freq, sample_rate):
    # Convert the cutoff frequency to the normalized frequency (0 < alpha < 1)
    RC = 1.0 / (2 * np.pi * cutoff_freq)
    dt = 1.0 / sample_rate
    alpha = dt / (RC + dt)
    return alpha

# Hard-coded low-pass filter implementation
def lowpass_filter(input_signal, alpha):
    output_signal = np.zeros_like(input_signal)
    output_signal[0] = input_signal[0]  # Initialize with the first input value

    # Apply the filter
    for n in range(1, len(input_signal)):
        output_signal[n] = alpha * output_signal[n-1] + (1 - alpha) * input_signal[n]

    return output_signal

############################ High Pass Filter ########################
# Hard-coded high-pass filter implementation
def highpass_filter(input_signal, alpha):
    output_signal = np.zeros_like(input_signal)
    output_signal[0] = input_signal[0]  # Initialize with the first input value

    # Apply the filter
    for n in range(1, len(input_signal)):
        output_signal[n] = alpha * (output_signal[n-1] + input_signal[n] - input_signal[n-1])

    return output_signal

############################ Band Pass Filter ########################
# Band-pass filter implementation using a combination of low-pass and high-pass filters
def bandpass_filter(input_signal, low_alpha, high_alpha):
    lowpassed = lowpass_filter(input_signal, low_alpha)
    bandpassed = highpass_filter(lowpassed, high_alpha)
    return bandpassed

############################ Notch Filter ############################
# Notch filter implementation using a combination of low-pass and high-pass filters
def notch_filter(input_signal, low_alpha, high_alpha):
    # Use high-pass and low-pass to create a band-stop (notch) filter
    lowpassed = lowpass_filter(input_signal, low_alpha)
    highpassed = highpass_filter(input_signal, high_alpha)
    output_signal = highpassed + lowpassed  # Create the notch filter effect
    return output_signal

############################ Input def #########################
# Generate a step input
def generate_step_input(length, step_point):
    input_signal = np.zeros(length)
    input_signal[step_point:] = 1
    return input_signal

############################### TESTING ##############################
# Test the filters with a step input
def test_filter():
    # Parameters
    signal_length = 100
    step_point = 20
    cutoff_freq_low_01 = 0.1  # Low-pass cutoff frequency in Hz
    cutoff_freq_low_10 = 1.0  # Low-pass cutoff frequency in Hz
    cutoff_freq_high_05 = 0.5  # High-pass cutoff frequency in Hz
    notch_freq_low = 0.4       # Lower cutoff for notch filter
    notch_freq_high = 0.6      # Upper cutoff for notch filter
    sample_rate = 1.0  # Sample rate in Hz

    # Time array based on signal length and sample rate
    time = np.arange(0, signal_length) / sample_rate

    # Generate step input
    step_input = generate_step_input(signal_length, step_point)

    # Low-pass filter
    alpha_low_01 = calculate_alpha(cutoff_freq_low_01, sample_rate)
    lowpass_filter01_output = lowpass_filter(step_input, alpha_low_01)
    
    alpha_low_10 = calculate_alpha(cutoff_freq_low_10, sample_rate)
    lowpass_filter10_output = lowpass_filter(step_input, alpha_low_10)

    # High-pass filter
    alpha_high_05 = calculate_alpha(cutoff_freq_high_05, sample_rate)
    highpass_filter_output = highpass_filter(step_input, alpha_high_05)

    # Band-pass filter (using low-pass 1 Hz and high-pass 0.1 Hz)
    alpha_bandpass_low = alpha_low_10
    alpha_bandpass_high = calculate_alpha(0.1, sample_rate)
    bandpass_filter_output = bandpass_filter(step_input, alpha_bandpass_low, alpha_bandpass_high)

    # Notch filter (using low-pass 0.6 Hz and high-pass 0.4 Hz)
    alpha_notch_low = calculate_alpha(notch_freq_low, sample_rate)
    alpha_notch_high = calculate_alpha(notch_freq_high, sample_rate)
    notch_filter_output = notch_filter(step_input, alpha_notch_low, alpha_notch_high)

    # Plot the input and output signals
    plt.figure(figsize=(12, 8))
    
    plt.plot(time, step_input, label='Step Input', linestyle='--', color='blue')

    # Low-pass filter outputs
    plt.plot(time, lowpass_filter01_output, label='Low Pass filter 0.1 [Hz]', color='red')
    plt.plot(time, lowpass_filter10_output, label='Low Pass filter 1 [Hz]', color='green')

    # High-pass filter output
    plt.plot(time, highpass_filter_output, label='High Pass filter 0.5 [Hz]', color='orange')

    # Band-pass filter output
    plt.plot(time, bandpass_filter_output, label='Band Pass filter 0.1-1 [Hz]', color='purple')

    # Notch filter output
    plt.plot(time, notch_filter_output, label='Notch filter 0.4-0.6 [Hz]', color='brown')

    plt.title('Manual Filter Responses to Step Input')
    plt.xlabel('Time [seconds]')
    plt.ylabel('Amplitude')
    plt.legend()
    plt.grid(True)
    plt.show()

# Run the test
test_filter()
