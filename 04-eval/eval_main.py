import numpy as np
import glob
from os.path import split
import matplotlib.pyplot as plt

# Parameters
folder_path = '/home/alexj/Dropbox/UFMG/04-2-2016/JornalPaper/03-results/2017-03-02_okayish/logs'
colors_robots=[(1., 0., 1.), (0., 0., 1.), (0., 1., 0.), (0.6, 0., 0.), (0.6, 0.6, 0.)]  # RGB
#                 Purple         Blue         Green          Red          Yellow



# Init
if folder_path[-1] is not '/':
    folder_path = folder_path+'/'

all_log_files = glob.glob(folder_path+'*.log')
all_log_files_fns = [split(s)[1] for s in all_log_files]
all_log_files_cut = [s[20:] for s in all_log_files_fns]
n_agents = len(all_log_files)/2

all_plnnr_files = [s for s in all_log_files_cut if s[:5] == 'plnnr']
all_cntrlr_files = [s for s in all_log_files_cut if s[:6] == 'cntrlr']
all_plnnr_files.sort()
all_cntrlr_files.sort()


# Planner evaluation
# init data structures for all agents
all_t = []
all_iter = []
all_winner_id = []
all_winner_cost = []
all_own_cost = []
all_nodes = []

# iterate through all planner logs
for i in range(n_agents):
    # open file of i-th planner
    fp = glob.glob(folder_path + '*' + all_plnnr_files[i])[0]
    fh = open(fp)
    content = []
    content = fh.readlines()
    content = [x.strip() for x in content]

    # lines of content
    m = len(content)-2 # -2 as the first two lines are header info

    # parse lines to arrays of floats
    t = np.zeros(m)
    iter = np.zeros(m)
    winner_id = np.zeros(m)
    winner_cost = np.zeros(m)
    own_cost = np.zeros(m)
    nodes = np.zeros(m)
    for j,line in enumerate(content[2:]):
        raw_data = line.split()
        t[j],iter[j],winner_id[j],winner_cost[j],own_cost[j],nodes[j]=[float(x) for x in raw_data]

    # append the individual sets to gather all data
    all_t.append(t)
    all_iter.append(iter)
    all_winner_id.append(winner_id)
    all_winner_cost.append(winner_cost)
    all_own_cost.append(own_cost)
    all_nodes.append(nodes)

# process data
normalized_t = all_t[0]
t_start = normalized_t[0]
normalized_t = [t-t_start for t in normalized_t]




#####################################################################################################################
# Plot Cost first 200s
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = 100
for i in range(n_agents):
    ax.plot(normalized_t[start:end], all_own_cost[i][start:end],color=colors_robots[i],label='robot '+str(i))
ax.plot(normalized_t[start:end], all_winner_cost[0][start:end],color='black',linewidth='2',label='consensus')

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('cost in s')
ax.set_xlabel('mission time in s')
fig1.subplots_adjust(bottom=0.14,left=0.175,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/cost_part.png', facecolor=fig1.get_facecolor(), transparent=True)

#####################################################################################################################
# Plot Cost all
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = None
for i in range(n_agents):
    ax.plot(normalized_t[start:end], all_own_cost[i][start:end],color=colors_robots[i],label='robot '+str(i))
ax.plot(normalized_t[start:end], all_winner_cost[0][start:end],color='black',linewidth='2',label='consensus')

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('cost in s')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[-1]])
fig1.subplots_adjust(bottom=0.14,left=0.175,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/cost_all.png', facecolor=fig1.get_facecolor(), transparent=True)

#####################################################################################################################
# Plot Nodes partially
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = 100
for i in range(n_agents):
    ax.plot(normalized_t[start:end], all_nodes[i][start:end],color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('number of nodes')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
fig1.subplots_adjust(bottom=0.14,left=0.175,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/nodes_part.png', facecolor=fig1.get_facecolor(), transparent=True)

#####################################################################################################################
# Plot Nodes all
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = None

if end is None:
    end = m-1

for i in range(n_agents):
    ax.plot(normalized_t[start:end], all_nodes[i][start:end],color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('number of nodes')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
fig1.subplots_adjust(bottom=0.14,left=0.175,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/nodes_all.png', facecolor=fig1.get_facecolor(), transparent=True)

#####################################################################################################################
# Plot Winner part
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = 100

# process data
new_t = [normalized_t[start]]
new_data = [all_winner_id[0][start]]
if end is None:
    end = m-1
for i in range(start+1,end):
    new_t.extend([normalized_t[i],normalized_t[i]])
    new_data.extend([all_winner_id[0][i-1],all_winner_id[0][i]])
new_t.append(new_t[-1]+normalized_t[start+1]-normalized_t[start])
new_data.append(new_data[-1])
#for i in range(n_agents):
ax.plot(new_t, new_data,color='black',label='winner id')

# decoration
#legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
#          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0,handlelength=1)
#for label in legend.get_texts():
#    label.set_fontsize(8)

ax.set_ylabel('winner id')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
ax.set_ylim([-0.1,n_agents-1+0.1])
fig1.subplots_adjust(bottom=0.14,left=0.125,right=0.95,top=0.97)
#plt.tight_layout()
fig1.savefig('output/winner_id_part.png', facecolor=fig1.get_facecolor(), transparent=True)


#####################################################################################################################
# Plot Winner part
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = None

# process data
new_t = [normalized_t[start]]
new_data = [all_winner_id[0][start]]
if end is None:
    end = m-1
for i in range(start+1,end):
    new_t.extend([normalized_t[i],normalized_t[i]])
    new_data.extend([all_winner_id[0][i-1],all_winner_id[0][i]])
new_t.append(new_t[-1]+normalized_t[start+1]-normalized_t[start])
new_data.append(new_data[-1])
#for i in range(n_agents):
ax.plot(new_t, new_data,color='black',label='winner id')

# decoration
#legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
#          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0,handlelength=1)
#for label in legend.get_texts():
#    label.set_fontsize(8)

ax.set_ylabel('winner id')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
ax.set_ylim([-0.1,n_agents-1+0.1])
fig1.subplots_adjust(bottom=0.14,left=0.125,right=0.95,top=0.97)
#plt.tight_layout()
fig1.savefig('output/winner_id_all.png', facecolor=fig1.get_facecolor(), transparent=True)



#####################################################################################################################
#####################################################################################################################
#####################################################################################################################
# controller evaluation

# init data structures for all agents
all_t = []
all_e_rho = []
all_e_phi = []
all_v = []
all_w = []

# iterate through all planner logs
for i in range(n_agents):
    # open file of i-th planner
    fp = glob.glob(folder_path + '*' + all_cntrlr_files[i])[0]
    fh = open(fp)
    content = fh.readlines()
    content = [x.strip() for x in content]

    # lines of content
    m = len(content)-2 # -2 as the first two lines are header info

    # parse lines to arrays of floats
    t = np.zeros(m)
    e_rho = np.zeros(m)
    e_phi = np.zeros(m)
    v = np.zeros(m)
    w = np.zeros(m)
    for j,line in enumerate(content[2:]):
        raw_data = line.split()
        t[j],e_rho[j],e_phi[j],v[j],w[j]=[float(x) for x in raw_data]

    # append the individual sets to gather all data
    all_t.append(t)
    all_e_rho.append(e_rho)
    all_e_phi.append(e_phi)
    all_v.append(v)
    all_w.append(w)

# process data
normalized_t = all_t[0]
t_start = normalized_t[0]
normalized_t = [t-t_start for t in normalized_t]







#####################################################################################################################
# Plot rho error
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = 100

if end is None:
    end = m-1

for i in range(n_agents):
    ax.plot(normalized_t[start:end], np.abs(all_e_rho[i][start:end]),color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('|e^i_rho| in m')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
fig1.subplots_adjust(bottom=0.14,left=0.2,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/e_rho_part.png', facecolor=fig1.get_facecolor(), transparent=True)

#####################################################################################################################
# Plot rho error all
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = None

if end is None:
    end = m-1

for i in range(n_agents):
    ax.plot(normalized_t[start:end], np.abs(all_e_rho[i][start:end]),color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('|e^i_rho| in m')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
fig1.subplots_adjust(bottom=0.14,left=0.2,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/e_rho_all.png', facecolor=fig1.get_facecolor(), transparent=True)



#####################################################################################################################
# Plot phi error
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = 50

if end is None:
    end = m-1

for i in range(n_agents):
    ax.plot(normalized_t[start:end], np.abs(all_e_phi[i][start:end]),color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('|e^i_phi| in rad')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
fig1.subplots_adjust(bottom=0.14,left=0.2,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/e_phi_part.png', facecolor=fig1.get_facecolor(), transparent=True)


#####################################################################################################################
# Plot phi error all
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = None

if end is None:
    end = m-1

for i in range(n_agents):
    ax.plot(normalized_t[start:end], np.abs(all_e_phi[i][start:end]),color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('|e^i_phi| in rad')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
fig1.subplots_adjust(bottom=0.14,left=0.2,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/e_phi_all.png', facecolor=fig1.get_facecolor(), transparent=True)

#####################################################################################################################
# Plot v
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = 100

if end is None:
    end = m-1

for i in range(n_agents):
    ax.plot(normalized_t[start:end], np.abs(all_v[i][start:end]),color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.67, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('v_i in cm/s')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
ax.set_ylim([0,max(all_v[i][start:end])*1.05])

fig1.subplots_adjust(bottom=0.14,left=0.2,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/v_part.png', facecolor=fig1.get_facecolor(), transparent=True)


#####################################################################################################################
# Plot v all
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = None

if end is None:
    end = m-1

for i in range(n_agents):
    ax.plot(normalized_t[start:end], np.abs(all_v[i][start:end]),color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.67, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('v_i in cm/s')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
ax.set_ylim([0,max(all_v[i][start:end])*1.05])

fig1.subplots_adjust(bottom=0.14,left=0.2,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/v_part.png', facecolor=fig1.get_facecolor(), transparent=True)


#####################################################################################################################
# Plot w
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = 100

if end is None:
    end = m-1

for i in range(n_agents):
    ax.plot(normalized_t[start:end], np.abs(all_w[i][start:end]),color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.67, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('|w_i| in rad/s')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
ax.set_ylim([0,max(all_w[i][start:end])*1.05])

fig1.subplots_adjust(bottom=0.14,left=0.2,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/w_part.png', facecolor=fig1.get_facecolor(), transparent=True)



#####################################################################################################################
# Plot w all
fig1 = plt.figure(facecolor='white')
fig1.set_figheight(3)
fig1.set_figwidth(3.5)
ax = fig1.add_subplot(1,1,1)

# generate plot for cost + consensus cost
start = 0
end = None

if end is None:
    end = m-1

for i in range(n_agents):
    ax.plot(normalized_t[start:end], np.abs(all_w[i][start:end]),color=colors_robots[i],label='robot '+str(i))

# decoration
legend = ax.legend(loc='upper center', bbox_to_anchor=(0.67, 1.1),
          ncol=2, fancybox=True, shadow=True,borderpad=0,labelspacing=0, handletextpad=0,handlelength=1,columnspacing=0.5)
for label in legend.get_texts():
    label.set_fontsize(8)

ax.set_ylabel('|w_i| in rad/s')
ax.set_xlabel('mission time in s')
ax.set_xlim([0,normalized_t[end]])
ax.set_ylim([0,max(all_w[i][start:end])*1.05])

fig1.subplots_adjust(bottom=0.14,left=0.2,right=0.95,top=0.95)
#plt.tight_layout()
fig1.savefig('output/w_all.png', facecolor=fig1.get_facecolor(), transparent=True)









