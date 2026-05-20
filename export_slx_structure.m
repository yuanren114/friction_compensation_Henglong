  model = 'friction_rt_online_aa05586_base';
  load_system(model);

  fprintf('\n=== All Blocks ===\n');
  blocks = find_system(model, 'Type', 'Block');
  for i = 1:numel(blocks)
      fprintf('%s\n', blocks{i});
  end

  fprintf('\n=== Subsystems ===\n');
  subs = find_system(model, 'BlockType', 'SubSystem');
  for i = 1:numel(subs)
      fprintf('%s\n', subs{i});
  end

  fprintf('\n=== Constants ===\n');
  consts = find_system(model, 'BlockType', 'Constant');
  for i = 1:numel(consts)
      fprintf('%s\n', consts{i});
      try
          fprintf('  Value: %s\n', get_param(consts{i}, 'Value'));
      catch
          fprintf('  Value: <unable to read>\n');
      end
  end

  fprintf('\n=== Delays ===\n');
  delays = find_system(model, 'BlockType', 'Delay');
  for i = 1:numel(delays)
      fprintf('%s\n', delays{i});
      try
          fprintf('  DelayLength: %s\n', get_param(delays{i}, 'DelayLength'));
      catch
      end
      try
          fprintf('  InitialConditionSource: %s\n', get_param(delays{i}, 'InitialConditionSource'));
      catch
      end
  end

  fprintf('\n=== Unit Delays ===\n');
  unit_delays = find_system(model, 'BlockType', 'UnitDelay');
  for i = 1:numel(unit_delays)
      fprintf('%s\n', unit_delays{i});
  end

  fprintf('\n=== Displays ===\n');
  disps = find_system(model, 'BlockType', 'Display');
  for i = 1:numel(disps)
      fprintf('%s\n', disps{i});
  end

  fprintf('\n=== Scopes ===\n');
  scopes = find_system(model, 'BlockType', 'Scope');
  for i = 1:numel(scopes)
      fprintf('%s\n', scopes{i});
  end

  fprintf('\n=== Inports ===\n');
  inports = find_system(model, 'BlockType', 'Inport');
  for i = 1:numel(inports)
      fprintf('%s\n', inports{i});
  end

  fprintf('\n=== Outports ===\n');
  outports = find_system(model, 'BlockType', 'Outport');
  for i = 1:numel(outports)
      fprintf('%s\n', outports{i});
  end

  fprintf('\n=== Subsystem Contents ===\n');
  for i = 1:numel(subs)
      fprintf('\n--- %s ---\n', subs{i});
      inside = find_system(subs{i}, 'SearchDepth', 1, 'Type', 'Block');
      for j = 1:numel(inside)
          fprintf('%s\n', inside{j});
      end
  end