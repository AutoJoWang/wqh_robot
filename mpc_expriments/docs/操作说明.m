# 运行实验
python3 mpc_expriments/mpc_liner_test.py --no-show --save mpc_expriments/docs/实验对比.png

#加弹窗
python3 mpc_expriments/mpc_liner_test.py --save mpc_expriments/docs/实验对比.png

#改参数
python3 mpc_expriments/mpc_liner_test.py \
  --x-end 12 --y-end 3 \
  --slips 0.5,0.6,0.7,0.8,0.9,1.0 \
  --save mpc_expriments/docs/实验对比.png

python3 mpc_expriments/mpc_liner_test.py --eta-low 0.4 --no-show   --save-prefix mpc_expriments/docs/阶跃扰动对比_0.4
#转向滑移侧漂对比
python3 mpc_expriments/mpc_s_test.py --no-show \
  --eta-omega 0.7 \
  --save-prefix mpc_expriments/docs/转向滑移侧漂对比