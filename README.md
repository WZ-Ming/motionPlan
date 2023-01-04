# 简介
编写了一种静态前瞻式7段S型速度规划，实现边计算边运动，并在运动过程中可平稳减速暂停以及启动。
目前算法在运动过程中不支持修改位移、速度、加速度等条件，只是是在这些条件确定下实现的前瞻速度规划。
## 有暂停情况
![image](https://user-images.githubusercontent.com/71966407/210471677-d0aefab9-9ba8-49b0-bdf9-286240e52c6a.png)

## 无暂停情况
![image](https://user-images.githubusercontent.com/71966407/210471748-ea088f10-6fc6-4c73-bc70-3c31f77a065b.png)
