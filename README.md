# React Native Facing Direction
This is a React Native nitro module that allows realtime updates for the magnetic cardinal direction the user is currently facing in.

# Permissiosn
No permissions are required.

# Install dependencies
`npm install react-native-nitro-modules`

## Installation
`npm install react-native-nitro-cardinal-direction`

## Usage

### Importing Module

`import { cardinalDirection } from 'react-native-nitro-cardinal-direction';`

### Starting the listener
```
cardinalDirection.startUpdates((data) => {
  console.log(data);
});
```

#### Output
```
{
  timestamp: number,
  heading: number,
  direction: string
}
```

### Stopping the listener

```
cardinalDirection.stopUpdates();
```
