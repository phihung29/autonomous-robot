import matplotlib.pylot as plt 
x_values= [0]
y_values= [0]
with open('record.txt','r') as file : 
    for line in file : 
        values= line.strip()[1:-1].split(',')
        x = float(values[0].strip())
        y = float(values[1].strip())
        x_values.append(x)
        y_values.append(y)

plt.figure()
plt.plot(x_values,y_values)
plt.show()

