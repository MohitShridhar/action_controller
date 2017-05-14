# Action Controller
Custom messages for dense refexp

# Installation

Clone and Make:
```bash
$ cd <your_catkin_ws>/src
$ https://github.com/MohitShridhar/action_controller.git

$ cd <your_catkin_ws>
$ catkin_make
```

# Usage

Check if query server is available:
```bash
$ rostopic list 
```
You should see these topic:
```
/dense_caption/cancel
/dense_caption/feedback
/dense_caption/goal
/dense_caption/result
/dense_caption/status
...
/dense_query/cancel
/dense_query/feedback
/dense_query/goal
/dense_query/result
/dense_query/status
```

Run Example:
```
cd <your_catkin_ws>/src/action_controller/test
python dense_query_example.py
```

See `dense_query_example.py` for usage


