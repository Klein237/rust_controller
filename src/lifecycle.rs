

#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum LifecycleState {
    Unconfigured,
    Inactive,
    Active,
    Finalized,
    Error,
}



pub struct LifecycleManager {
    state: LifecycleState,
    
}

impl LifecycleManager {
    pub fn new() -> Self {
        Self {
            state: LifecycleState::Unconfigured,
            
        }
    }

    // This function is used to get the current state of the lifecycle manager
    pub fn get_state(&self) -> LifecycleState {
        self.state
    }

    // This function allows the lifecycle manager to transition to the configured state
    fn configure(&mut self) -> Result<(), String> {
        if self.state != LifecycleState::Unconfigured {
            return Err("Cannot configure from current state".to_string());
        }
        
        self.state = LifecycleState::Inactive;
        Ok(())
    }

    // This function allows the lifecycle manager to transition to the active state
    fn activate(&mut self) -> Result<(), String> {
        if self.state != LifecycleState::Inactive {
            return Err("Cannot activate from current state".to_string());
        }
        
        self.state = LifecycleState::Active;
        Ok(())
    }

    // This function allows the lifecycle manager to transition to the inactive state
    fn deactivate(&mut self) -> Result<(), String> {
        if self.state != LifecycleState::Active {
            return Err("Cannot deactivate from current state".to_string());
        }
        
        self.state = LifecycleState::Inactive;
        Ok(())
    }

    // This function allows the lifecycle manager to transition to the finalized state
    fn cleanup(&mut self) -> Result<(), String> {
        if self.state == LifecycleState::Active {
            return Err("Cannot cleanup from Active state".to_string());
        }
        
        self.state = LifecycleState::Finalized;
        Ok(())
    }

    // This function allows the lifecycle manager to transition automatically between states
    // based on the current state and the target state
    pub fn transition(&mut self, target: LifecycleState) -> Result<(), String> {
        match (self.get_state(), target) {
            (LifecycleState::Unconfigured, LifecycleState::Inactive) => self.configure(),
            (LifecycleState::Inactive, LifecycleState::Active) => self.activate(),
            (LifecycleState::Active, LifecycleState::Inactive) => self.deactivate(),
            (LifecycleState::Inactive, LifecycleState::Finalized) => self.cleanup(),
            _ => Err("Transition not allowed.".to_string())
        }
    }
    
}
