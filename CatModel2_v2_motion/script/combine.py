'''
Combined imitation learning data and interpolate the data.
'''
import pandas as pd
import numpy as np 
import math
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

def estimate_time_to_target(desired_base_displacement):
    dx = desired_base_displacement[0]
    dy = desired_base_displacement[1]
    dyaw = desired_base_displacement[2]
    
    # 计算旋转时间和位移时间
    rotation_time = abs(dyaw) / TARGET_ROTATION_VELOCITY
    displacement = np.sqrt(dx**2 + dy**2)
    displacement_time = displacement / TARGET_DISPLACEMENT_VELOCITY
    
    # 返回旋转时间和位移时间中的较大值
    return max(rotation_time, displacement_time)

def read_file(state_file, euler_file, joint_file, foot_file):
    # 读取每个文件的内容，并指定空格为分隔符
    base_state = pd.read_csv(state_file, sep=' ', header=None)
    base_euler = pd.read_csv(euler_file, sep=' ', header=None)
    base_euler = base_euler * np.pi / 180
    joint_pos = pd.read_csv(joint_file, sep=' ', header=None)
    foot_pos = pd.read_csv(foot_file, sep=' ', header=None)

    # 合并三个数据帧
    combined_df = pd.concat([base_state, base_euler, joint_pos, foot_pos], axis=1)

    # 为每一列生成合适的列名
    base_state.columns = ['base_positionInWorld_x', 'base_positionInWorld_y', 'base_positionInWorld_z']
    base_euler.columns = ['base_euler_x', 'base_euler_y', 'base_euler_z']
    # 取决于所提供轨迹的关节顺序，不是最后希望得到的顺序
    joint_pos.columns = ['jointAngle_yaw',  # base
                         'jointAngle_LH_HAA', 'jointAngle_LH_HFE', 'jointAngle_LH_KFE',
                         'jointAngle_RH_HAA', 'jointAngle_RH_HFE', 'jointAngle_RH_KFE',
                         'jointAngle_pitch', # chest
                         'jointAngle_LF_HAA', 'jointAngle_LF_HFE', 'jointAngle_LF_KFE',
                         'jointAngle_RF_HAA', 'jointAngle_RF_HFE', 'jointAngle_RF_KFE']
    foot_pos.columns = ['LF_x', 'LF_y', 'LF_z',
                        'RF_x', 'RF_y', 'RF_z',
                        'LH_x', 'LH_y', 'LH_z',
                        'RH_x', 'RH_y', 'RH_z']

    # 合并后重新设置列名
    combined_df.columns = base_state.columns.tolist() + base_euler.columns.tolist() + joint_pos.columns.tolist() \
                          + foot_pos.columns.tolist()
    
    return combined_df

def add_timestamp(combined_df):
    timestamps = [0]  # 从 0 开始的时间戳
    for i in range(1, len(combined_df)):
        # 计算相邻两行的位移（假设 base_state 和 base_euler 是相邻行的数据）
        displacement = [
            combined_df.loc[i, 'base_positionInWorld_x'] - combined_df.loc[i - 1, 'base_positionInWorld_x'],
            combined_df.loc[i, 'base_positionInWorld_y'] - combined_df.loc[i - 1, 'base_positionInWorld_y'],
            combined_df.loc[i, 'base_euler_z'] - combined_df.loc[i - 1, 'base_euler_z']  # 假设 z 表示航向角
        ]
        
        # 估算时间差并更新时间戳
        time_diff = estimate_time_to_target(displacement)
        timestamps.append(timestamps[-1] + time_diff)

    # 将时间戳列添加到数据表中
    combined_df['time'] = timestamps

    # 更改列的顺序 -- [最终希望的顺序]
    cols = ['time', 
            'contactflag_LF', 'contactflag_RF', 'contactflag_LH', 'contactflag_RH',
            'base_positionInWorld_x', 'base_positionInWorld_y', 'base_positionInWorld_z', 
            'base_euler_x', 'base_euler_y', 'base_euler_z',
            'jointAngle_LF_HAA', 'jointAngle_LF_HFE', 'jointAngle_LF_KFE',
            'jointAngle_RF_HAA', 'jointAngle_RF_HFE', 'jointAngle_RF_KFE',
            'jointAngle_pitch',
            'jointAngle_yaw', 
            'jointAngle_LH_HAA', 'jointAngle_LH_HFE', 'jointAngle_LH_KFE',
            'jointAngle_RH_HAA', 'jointAngle_RH_HFE', 'jointAngle_RH_KFE',
        ]
    combined_df = combined_df[cols]

    return combined_df

def interpolate_by_col(df, threshold, col):
    df = df.copy()
    result = []

    for i in range(len(df) - 1):
        # 当前行和下一行
        row_start = df.iloc[i]
        row_end = df.iloc[i + 1]
        result.append(row_start)  # 先添加当前行
        
        # 检查是否需要插值
        diff = abs(row_end[col] - row_start[col])
        if diff >= threshold:
            # 计算需要插入的新点数
            num_new_points = int(diff // threshold)
            for j in range(1, num_new_points + 1):
                ratio = j / (num_new_points + 1)
                new_row = row_start + (row_end - row_start) * ratio
                result.append(new_row)  # 插入新行

    # 添加最后一行
    result.append(df.iloc[-1])

    # 转换结果为 DataFrame
    result_df = pd.DataFrame(result).reset_index(drop=True)
    return result_df

def get_rotation_matrix(eulerAngles):
    x, y, z = eulerAngles

    c1, c2, c3 = np.cos(z), np.cos(y), np.cos(x)
    s1, s2, s3 = np.sin(z), np.sin(y), np.sin(x)

    s2s3 = s2 * s3
    s2c3 = s2 * c3

    # 生成旋转矩阵
    rotation_matrix = np.array([
        [c1 * c2, c1 * s2s3 - s1 * c3, c1 * s2c3 + s1 * s3],
        [s1 * c2, s1 * s2s3 + c1 * c3, s1 * s2c3 - c1 * s3],
        [-s2,     c2 * s3,             c2 * c3]
    ])
    
    return rotation_matrix

def get_foot_pos_in_world(df):
    base_state = df[['base_positionInWorld_x', 'base_positionInWorld_y', 'base_positionInWorld_z']]
    base_euler = df[['base_euler_x', 'base_euler_y', 'base_euler_z']]
    foot_pos = df[['LF_x', 'LF_y', 'LF_z',
                   'RF_x', 'RF_y', 'RF_z',
                   'LH_x', 'LH_y', 'LH_z',
                   'RH_x', 'RH_y', 'RH_z']]
    foot_pos_in_world = np.array(foot_pos)
    for i in range(len(foot_pos)):
        # 生成旋转矩阵
        rotation_matrix = get_rotation_matrix(base_euler.loc[i].values)
        # 计算脚的位置
        for j in range(4):
            foot_pos_in_world[i, (3 * j):(3 * j + 3)] = np.dot(rotation_matrix, foot_pos.iloc[i, j * 3:(j + 1) * 3].values)
            foot_pos_in_world[i, (3 * j):(3 * j + 3)] += base_state.loc[i].values
            
    return foot_pos_in_world

def judge_stance(foot_pos_in_world):
    foot_pos_in_world = pd.DataFrame(foot_pos_in_world)
    contact_flag = np.zeros((len(foot_pos_in_world), 4))
    z_LF = foot_pos_in_world.iloc[:, 2]
    z_RF = foot_pos_in_world.iloc[:, 5]
    z_LH = foot_pos_in_world.iloc[:, 8]
    z_RH = foot_pos_in_world.iloc[:, 11]
    
    
    begin = []
    crossings = np.where(np.diff(np.sign(z_LF - z_RF)) != 0)[0]  # 交叉点索引
    begin.append(crossings[0])
    for i in range(len(crossings) - 1):
        start, end = crossings[i], crossings[i + 1]
        if np.mean(z_LF[start:end]) < np.mean(z_RF[start:end]):
            contact_flag[start:end, 0] = 1  # 左前腿站立
        else:
            contact_flag[start:end, 1] = 1  # 右前腿站立
    
    crossings = np.where(np.diff(np.sign(z_LH - z_RH)) != 0)[0]  # 交叉点索引
    begin.append(crossings[0])
    for i in range(len(crossings) - 1):
        start, end = crossings[i], crossings[i + 1]
        if np.mean(z_LH[start:end]) < np.mean(z_RH[start:end]):
            contact_flag[start:end, 2] = 1  # 左后腿站立
        else:
            contact_flag[start:end, 3] = 1  # 右后腿站立
            
    # 取begin大的那个，之前的全部设为1
    max_begin = max(begin)
    for i in range(max_begin):
        contact_flag[i, :] = 1
        
    # 约束限制
    # 对于每个时刻，要么两个对立的腿支撑地面，要么四个腿都支撑地面
    record = []
    for i in range(len(contact_flag)):
        if sum(contact_flag[i, :]) == 4:
            continue
        if sum(contact_flag[i, :]) == 2:
            if contact_flag[i, 0] == contact_flag[i, 3] and contact_flag[i, 1] == contact_flag[i, 2]:
                continue
            else:
                contact_flag[i, :] = 1
        if sum(contact_flag[i, :]) == 3:
            contact_flag[i, :] = 1
            record.append(i)
        if sum(contact_flag[i, :]) == 0: # 对应最后的一段
            contact_flag[i, :] = 1
            
        
    # 绘制波谷
    # plt.plot(foot_pos_in_world.iloc[:, 2], label='LF')
    # plt.plot(foot_pos_in_world.iloc[:, 5], label='RF')
    # plt.plot(foot_pos_in_world.iloc[:, 8], label='LH')
    # plt.plot(foot_pos_in_world.iloc[:, 11], label='RH')
    # plt.scatter(record, foot_pos_in_world.iloc[record, 2], c='r', label='valleys')
    # plt.legend()
    # plt.show()
    return contact_flag

def draw_foot_pos(foot_pos_in_world):
    base_positon = pd.DataFrame(foot_pos_in_world[:, :3])
    foot_pos_in_world = pd.DataFrame(foot_pos_in_world)
    # plt.plot(foot_pos_in_world.iloc[:, 0], foot_pos_in_world.iloc[:, 1], label='LF')
    # plt.plot(foot_pos_in_world.iloc[:, 3], foot_pos_in_world.iloc[:, 4], label='RF')
    # plt.plot(foot_pos_in_world.iloc[:, 6], foot_pos_in_world.iloc[:, 7], label='LH')
    # plt.plot(foot_pos_in_world.iloc[:, 9], foot_pos_in_world.iloc[:, 10], label='RH')
    # plt.legend()
    # plt.show()
    
    # 绘制世界系x坐标变化
    plt.plot(base_positon.iloc[:, 0], base_positon.iloc[:, 1], marker='o', linestyle='None')

    # 画所有点
    plt.scatter(base_positon.iloc[0, 0], base_positon.iloc[0, 1], c='r', label='start')
    # 绘制四条腿的z坐标变化
    # plt.plot(foot_pos_in_world.iloc[:, 2], label='LF')
    # plt.plot(foot_pos_in_world.iloc[:, 5], label='RF')
    # plt.plot(foot_pos_in_world.iloc[:, 8], label='LH')
    # plt.plot(foot_pos_in_world.iloc[:, 11], label='RH')
    plt.legend()
    plt.show()

def add_contact_flag(df):
    foot_pos_in_world = get_foot_pos_in_world(df)
    draw_foot_pos(foot_pos_in_world)
    contact_flag = judge_stance(foot_pos_in_world)
    contact_flag = pd.DataFrame(contact_flag)
    contact_flag.columns = ['contactflag_LF', 'contactflag_RF', 'contactflag_LH', 'contactflag_RH']
    contact_flag.to_csv('contact_flag.csv', index=False)
    # 舍弃原始的脚部位置数据
    df = df.drop(['LF_x', 'LF_y', 'LF_z',
                  'RF_x', 'RF_y', 'RF_z',
                  'LH_x', 'LH_y', 'LH_z',
                  'RH_x', 'RH_y', 'RH_z'], axis=1)
    # 将接触状态数据添加到数据表中
    df = pd.concat([df, contact_flag], axis=1)
    return df

def judge_center_of_each_circle(df):
    base_x = df['base_positionInWorld_x']
    base_y = df['base_positionInWorld_y']

    peaks_down, _ = find_peaks(-base_y)  # 波谷
    peaks_up,   _ = find_peaks( base_y)  # 波峰
    plt.plot(base_x, base_y)
    plt.plot(base_x[peaks_down], base_y[peaks_down], 'ro')  # 用红色标记波谷
    plt.plot(base_x[peaks_up  ], base_y[peaks_up  ], 'go')  # 用绿色标记波峰
    plt.show()

    # 中心坐标

    print("Center of each circle: ")
    for i in range(len(peaks_down)):
        print(round((base_x[peaks_down[i]]), 2), round((base_x[peaks_up[i]]), 2))
    

def write_file(interpolated_df, filename):

    interpolated_df.to_csv(filename, index=False)
    with open(filename, 'r') as file:
        content = file.read().replace(',', ', ')
    with open(filename, 'w') as file:
        file.write(content)
        
if __name__ == '__main__':
    # file path
    state_file = 'new/root_state.txt'
    euler_file = 'new/root_euler_angles.txt'
    joint_file = 'new/q_final.txt'
    foot_file = 'new/foot_pos.txt'
    output_file = 'combined_output_interpolated.csv'
    # threshold
    # threshold = 0.002
    # 定义目标旋转速度和位移速度，来自 reference.info
    TARGET_ROTATION_VELOCITY = 0.78 # new: 1.27 
    TARGET_DISPLACEMENT_VELOCITY = 0.3 # old: 0.8 
    comHeight = 0.318 # old: 0.26
    
    combined_df = read_file(state_file, euler_file, joint_file, foot_file)
    
    # 使用滑动窗口对所有列进行平滑处理
    combined_df[:500] = combined_df[:500].rolling(window=30, min_periods=1).mean()

    # 依次对每一列进行插值
    # for col in combined_df.columns:
    #     if col not in ['base_euler_z',
    #                    'jointAngle_LF_HAA', 'jointAngle_LF_HFE', 'jointAngle_LF_KFE',
    #                    'jointAngle_RF_HAA', 'jointAngle_RF_HFE', 'jointAngle_RF_KFE',
    #                    'jointAngle_pitch',  'jointAngle_yaw',
    #                    'jointAngle_LH_HAA', 'jointAngle_LH_HFE', 'jointAngle_LH_KFE',
    #                    'jointAngle_RH_HAA', 'jointAngle_RH_HFE', 'jointAngle_RH_KFE',
    #                 #    'base_positionInWorld_x', 'base_positionInWorld_y'
    #                    ]:
    #         continue
    #     interpolated_df = interpolate_by_col(combined_df, threshold, col)
    #     combined_df = interpolated_df
        
    combined_df = add_contact_flag(combined_df)
    combined_df = add_timestamp(combined_df)
    # 只取前1500行
    # combined_df = combined_df[:5120]
    write_file(combined_df, output_file)

    # with open('combined_output_interpolated.csv', 'r') as file:
    #     combined_df = pd.read_csv(file, sep=', ')

    # judge_center_of_each_circle(combined_df)
    
        
    
