import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def generate_random_multisine_timeseries(length=100):

    t = np.linspace(0, 2 * np.pi, length)
    
    frequncies = np.array([0.05,0.1,0.5,1])

    # Generate a multisine time series by summing sine waves with random frequencies and amplitudes
    multisine_series = np.zeros(length)
    for frequency in frequncies:
        amplitude = np.random.uniform(0.1, 1)  # Random amplitude between 0.1 and 1
        multisine_series += amplitude * np.sin(frequency * t * 2 * np.pi)
    
    return t, multisine_series

def generate_reference_and_limits(duration, scale):

    reference = scale*generate_random_multisine_timeseries(duration)[1]

    bound = 3*scale
    upper_margin = bound*np.ones(duration)
    lower_margin = bound*np.ones(duration)
    alpha = 0.6
    for t in range(duration-1):
        upper_margin[t+1] = alpha*upper_margin[t] + (1-alpha)*np.random.uniform(0.5, bound)
        lower_margin[t+1] = alpha*lower_margin[t] + (1-alpha)*np.random.uniform(0.5, bound)

    upper = reference + upper_margin
    lower = reference - lower_margin

    return reference, upper, lower
    
def plot_reference_and_terrain(reference, upper, lower):
    plt.plot(reference, 'b')
    plt.plot(upper, 'r')
    plt.plot(lower, 'r')
    plt.show()

def write_mission_to_csv(mission, file_name):
    # Create a DataFrame from the Mission object's attributes
    data = {
        'reference': mission.reference,
        'cave_height': mission.cave_height,
        'cave_depth': mission.cave_depth
    }
    df = pd.DataFrame(data)
    
    # Write the DataFrame to a CSV file
    df.to_csv(file_name, index=False)


    