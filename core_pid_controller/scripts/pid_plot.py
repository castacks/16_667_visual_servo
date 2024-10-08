#!/usr/bin/python
import rosbag
import sys
import collections
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    bag_filename = sys.argv[1]

    data = collections.OrderedDict()
    gains = collections.OrderedDict()

    initial_time = None
    
    bag = rosbag.Bag(bag_filename, 'r')
    for topic, msg, t in bag.read_messages():
        if 'pid_info' in topic:
            if initial_time == None:
                initial_time = msg.header.stamp.to_sec()
            
            names = filter(lambda x:x!='', topic.split('/'))
            variable_name = names[-2]
            group_name = names[-3]

            # initialize group
            if group_name not in data.keys():
                data[group_name] = collections.OrderedDict()
                gains[group_name] = collections.OrderedDict()
            
            # initialize variable data
            if variable_name not in data[group_name].keys():
                data[group_name][variable_name] = {'time': [],
                                                   'p_component': [],
                                                   'i_component': [],
                                                   'd_component': [],
                                                   'ff_component': [],
                                                   'constant': [],
                                                   'control': [],
                                                   'target': [],
                                                   'actual': [],
                                                   'error': []}
                gains[group_name][variable_name] = {'P': msg.P,
                                                    'I': msg.I,
                                                    'D': msg.D,
                                                    'FF': msg.FF,
                                                    'neg_P': msg.neg_P,
                                                    'neg_I': msg.neg_I,
                                                    'neg_D': msg.neg_D,
                                                    'neg_FF': msg.neg_FF,
                                                    'min': msg.min,
                                                    'max': msg.max}
            # add data
            data[group_name][variable_name]['time'].append(msg.header.stamp.to_sec() - initial_time)
            data[group_name][variable_name]['p_component'].append(msg.p_component)
            data[group_name][variable_name]['i_component'].append(msg.i_component)
            data[group_name][variable_name]['d_component'].append(msg.d_component)
            data[group_name][variable_name]['ff_component'].append(msg.ff_component)
            data[group_name][variable_name]['constant'].append(msg.constant)
            data[group_name][variable_name]['control'].append(msg.control)
            data[group_name][variable_name]['target'].append(msg.target)
            data[group_name][variable_name]['actual'].append(msg.actual)
            data[group_name][variable_name]['error'].append(msg.error)
            

    plots = {}
    
    for group in data.keys():
        # initialize the plots
        if group not in plots.keys():
            values_fig = plt.figure()
            values_fig.suptitle(group)
            components_fig = plt.figure()
            components_fig.suptitle(group)
            plots[group] = {'values': values_fig, 'components': components_fig}

        # figure out the number of subplots needed
        total_subplots = len(data[group].keys())
        if np.sqrt(total_subplots) != np.round(np.sqrt(total_subplots)):
            total_subplots = int(np.round(np.sqrt(total_subplots)+1)**2)
        rows_cols = int(np.sqrt(total_subplots))

        # plot
        for i in range(len(data[group].keys())):
            # plot actual and target values
            variable_name = data[group].keys()[i]
            fig = plots[group]['values']
            ax = fig.add_subplot(rows_cols, rows_cols, i+1)
            ax.set_title(variable_name + '\nP: %0.2f, I: %0.2f, D: %0.2f, FF: %0.2f' % (gains[group][variable_name]['P'],
                                                                                        gains[group][variable_name]['I'],
                                                                                        gains[group][variable_name]['D'],
                                                                                        gains[group][variable_name]['FF']))
            ax.plot(data[group][variable_name]['time'], data[group][variable_name]['target'], 'b-', label='target')
            ax.plot(data[group][variable_name]['time'], data[group][variable_name]['actual'], 'r-', label='actual')
            ax.legend()

            # plot p, i, d, ff components
            fig = plots[group]['components']
            ax = fig.add_subplot(rows_cols, rows_cols, i+1)
            ax.set_title(variable_name + '\nP: %0.2f, I: %0.2f, D: %0.2f, FF: %0.2f' % (gains[group][variable_name]['P'],
                                                                                        gains[group][variable_name]['I'],
                                                                                        gains[group][variable_name]['D'],
                                                                                        gains[group][variable_name]['FF']))
            ax.plot(data[group][variable_name]['time'], data[group][variable_name]['p_component'], 'r-', label='P')
            ax.plot(data[group][variable_name]['time'], data[group][variable_name]['i_component'], 'g-', label='I')
            ax.plot(data[group][variable_name]['time'], data[group][variable_name]['d_component'], 'b-', label='D')
            ax.plot(data[group][variable_name]['time'], data[group][variable_name]['ff_component'], 'y-', label='FF')
            ax.plot(data[group][variable_name]['time'], data[group][variable_name]['control'], 'm-', label='control')
            ax.legend()


    plt.show()
