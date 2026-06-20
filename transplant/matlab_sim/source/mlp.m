% filepath: source/train_mlp_from_timetable.m
function train_mlp_from_timetable()

%% 1) 路径与超参数
dataDir = fullfile(fileparts(mfilename('fullpath')), '..', 'tran_data'); % source/../tran_data
hiddenSizes = [64 64];        % MLP结构，可改
trainRatio = 0.8;
valRatio   = 0.1;
testRatio  = 0.1;
rng(42);                      % 固定随机种子

%% 2) 加载数据（N x 8）
Xy = load_all_samples_from_tran_data(dataDir);  % N x 8
assert(size(Xy,2) == 8, '数据列数必须为8（前6列特征，后2列标签）');

X = Xy(:,1:6)';   % 神经网络工具箱常用: 特征维 x 样本数
Y = Xy(:,7:8)';   % 标签维 x 样本数

%% 3) 归一化（建议）
[Xn, psX] = mapminmax(X, -1, 1);
[Yn, psY] = mapminmax(Y, -1, 1);

%% 4) 构建并训练 MLP
net = fitnet(hiddenSizes, 'trainlm');   % 回归任务常用trainlm
net.layers{end}.transferFcn = 'purelin';% 输出层线性
net.divideFcn = 'dividerand';
net.divideParam.trainRatio = trainRatio;
net.divideParam.valRatio   = valRatio;
net.divideParam.testRatio  = testRatio;

net.trainParam.epochs = 1000;
net.trainParam.max_fail = 20;
net.trainParam.min_grad = 1e-7;
net.performFcn = 'mse';

[net, tr] = train(net, Xn, Yn);

%% 5) 评估
Yp_n = net(Xn);
Yp   = mapminmax('reverse', Yp_n, psY);   % 反归一化到真实量纲

mse_all = mean((Yp(:)-Y(:)).^2);
mae_all = mean(abs(Yp(:)-Y(:)));

fprintf('训练完成。\n');
fprintf('总样本数: %d\n', size(X,2));
fprintf('MSE(all): %.6g\n', mse_all);
fprintf('MAE(all): %.6g\n', mae_all);
fprintf('Train/Val/Test = %d / %d / %d\n', numel(tr.trainInd), numel(tr.valInd), numel(tr.testInd));

%% 6) 保存模型
savePath = fullfile(fileparts(mfilename('fullpath')), 'mlp_mpc_model.mat');
save(savePath, 'net', 'psX', 'psY', 'tr');
fprintf('模型已保存: %s\n', savePath);

%% 7) 可视化（可选）
figure('Name','Prediction vs Target');
subplot(2,1,1);
plot(Y(1,:), 'b'); hold on; plot(Yp(1,:), 'r--');
legend('Target y1','Pred y1'); grid on; title('Output 1');

subplot(2,1,2);
plot(Y(2,:), 'b'); hold on; plot(Yp(2,:), 'r--');
legend('Target y2','Pred y2'); grid on; title('Output 2');

end

function Xy = load_all_samples_from_tran_data(dataDir)
% 从tran_data目录读取所有.mat，提取时间表/矩阵，拼接成N x 8
if ~isfolder(dataDir)
    error('找不到数据文件夹: %s', dataDir);
end

files = dir(fullfile(dataDir, '*.mat'));
if isempty(files)
    error('tran_data中没有.mat文件: %s', dataDir);
end

allRows = [];

for i = 1:numel(files)
    fpath = fullfile(files(i).folder, files(i).name);
    S = load(fpath);
    vars = fieldnames(S);

    found = false;
    for k = 1:numel(vars)
        v = S.(vars{k});

        % 情况1: timetable
        if istimetable(v)
            A = timetable2array(v);
            if size(A,2) == 8
                allRows = [allRows; A]; %#ok<AGROW>
                found = true;
                break;
            end
        end

        % 情况2: table
        if istable(v)
            A = table2array(v);
            if size(A,2) == 8
                allRows = [allRows; A]; %#ok<AGROW>
                found = true;
                break;
            end
        end

        % 情况3: 数值矩阵
        if isnumeric(v) && ismatrix(v) && size(v,2) == 8
            allRows = [allRows; v]; %#ok<AGROW>
            found = true;
            break;
        end
    end

    if ~found
        warning('文件未找到8列数据，已跳过: %s', files(i).name);
    end
end

if isempty(allRows)
    error('未从tran_data提取到有效8列样本。');
end

% 清理异常值
allRows = allRows(all(isfinite(allRows),2), :);
Xy = allRows;
end