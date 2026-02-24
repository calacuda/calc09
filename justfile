export SESSION := "calc09"

_:
  @just -l

kill-docs:
  #!/usr/bin/bash
  proc=$(ps aux | rg "python3 -m http.server" | rg -v "rg python3" | cut -d " " -f 4)
  kill $proc

docs-is-running:
  ps aux | rg "python3 -m http.server" | rg -v "rg python3"
  echo "if there is no output above, server is not running"  

server-docs:
  #!/usr/bin/bash
  proc=$(ps aux | rg "python3 -m http.server" | rg -v "rg python3" | cut -d " " -f 4)
  dir=$(pwdx $proc 2> /dev/null | cut -d ' ' -f 2)

  if [[ -z $proc ]]; then
    echo "no docs server found; RUNNING"
    # nohup python3 -m http.server -b 127.0.0.1 -d ./target/doc 8080 > /dev/null &
    nohup python3 -m http.server -d ./target/doc 8080 > /dev/null &
    echo "docs-server process: $(ps aux | rg "python3 -m http.server" | rg -v "rg python3" | cut -d " " -f 4)"
  elif [[ $dir == $PWD && -n $dir ]]; then
    echo "server found to be already running; NOT RUNNING"
    echo "docs server running in: $dir"
    echo "docs-server process: $proc"
  else
    echo "docks server running in another directory; won't disturb"
    echo "docs server running in: $dir"
    echo "docs-server process: $proc"
  fi

open-docs:
  $BROWSER localhost:8080

[parallel]
docs: server-docs open-docs

_new-tmux-dev-session:
  tmux new -ds "$SESSION" -n "README"
  tmux send-keys -t "$SESSION":README 'nv ./README.md "+set wrap"' ENTER
  @just _new-window "Edit" "nv ./src/*.rs"
  @just _new-window "Cargo" ""
  @just _new-window "Misc" ""
  @just _new-window "Git" "git status"
  
_new-window NAME CMD:
  tmux new-w -t "$SESSION" -n "{{NAME}}"
  [[ "{{CMD}}" != "" ]] && tmux send-keys -t "$SESSION":"{{NAME}}" "{{CMD}}" ENTER || true

tmux:
  tmux has-session -t "=$SESSION" || just _new-tmux-dev-session
  tmux a -t "=$SESSION"
