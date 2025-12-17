/**
 * Signup Form Component
 * Purpose: Multi-step wizard with email/password and background questionnaire
 * Date: 2025-12-14
 */

import React, { useState } from 'react';
import { authClient } from '../../lib/auth';
import { useSession } from './AuthProvider';
import styles from './AuthForms.module.css';

interface SignupFormData {
  email: string;
  password: string;
  confirmPassword: string;
  full_name: string;
  experience_level: string;
  programming_languages: string[];
  frameworks: string[];
  available_hardware: string[];
  robotics_hardware: string[];
}

const EXPERIENCE_LEVELS = ['Beginner', 'Intermediate', 'Advanced', 'Expert'];

const PROGRAMMING_LANGUAGES = [
  'Python',
  'JavaScript/TypeScript',
  'C++',
  'C',
  'Java',
  'Rust',
  'Go',
  'Other',
];

const FRAMEWORKS = [
  'React',
  'FastAPI',
  'Django',
  'TensorFlow',
  'PyTorch',
  'ROS',
  'ROS2',
  'OpenCV',
  'Other',
];

const HARDWARE_OPTIONS = [
  'CPU (x86/ARM)',
  'NVIDIA GPU',
  'AMD GPU',
  'Apple Silicon (M1/M2/M3)',
  'Raspberry Pi',
  'Jetson Nano/Xavier',
  'TPU',
  'Other',
];

const ROBOTICS_HARDWARE = [
  'Arduino',
  'ESP32/ESP8266',
  'Raspberry Pi',
  'NVIDIA Jetson',
  'Servo Motors',
  'Stepper Motors',
  'IMU/Gyroscope',
  'Camera/Vision Sensors',
  'LIDAR',
  'Ultrasonic Sensors',
  'None',
  'Other',
];

interface SignupFormProps {
  onSuccess?: () => void;
}

export const SignupForm: React.FC<SignupFormProps> = ({ onSuccess }) => {
  const { refreshSession } = useSession();
  const [step, setStep] = useState(1);
  const [formData, setFormData] = useState<SignupFormData>({
    email: '',
    password: '',
    confirmPassword: '',
    full_name: '',
    experience_level: '',
    programming_languages: [],
    frameworks: [],
    available_hardware: [],
    robotics_hardware: [],
  });
  const [error, setError] = useState('');
  const [isLoading, setIsLoading] = useState(false);

  /**
   * Handle input changes for text fields
   */
  const handleChange = (e: React.ChangeEvent<HTMLInputElement | HTMLSelectElement>) => {
    const { name, value } = e.target;
    setFormData((prev) => ({ ...prev, [name]: value }));
    setError('');
  };

  /**
   * Handle checkbox changes for multi-select fields
   */
  const handleCheckboxChange = (field: keyof SignupFormData, value: string) => {
    setFormData((prev) => {
      const currentValues = prev[field] as string[];
      const newValues = currentValues.includes(value)
        ? currentValues.filter((v) => v !== value)
        : [...currentValues, value];
      return { ...prev, [field]: newValues };
    });
  };

  /**
   * Validate Step 1: Account Details
   */
  const validateStep1 = (): boolean => {
    if (!formData.email || !formData.password || !formData.confirmPassword || !formData.full_name) {
      setError('All fields are required');
      return false;
    }

    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters');
      return false;
    }

    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return false;
    }

    const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
    if (!emailRegex.test(formData.email)) {
      setError('Invalid email address');
      return false;
    }

    return true;
  };

  /**
   * Validate Step 2: Software Background
   */
  const validateStep2 = (): boolean => {
    if (!formData.experience_level) {
      setError('Please select your experience level');
      return false;
    }

    if (formData.programming_languages.length === 0) {
      setError('Please select at least one programming language');
      return false;
    }

    return true;
  };

  /**
   * Handle next step
   */
  const handleNext = () => {
    if (step === 1 && !validateStep1()) return;
    if (step === 2 && !validateStep2()) return;

    setStep((prev) => prev + 1);
    setError('');
  };

  /**
   * Handle previous step
   */
  const handleBack = () => {
    setStep((prev) => prev - 1);
    setError('');
  };

  /**
   * Handle form submission
   */
  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!validateStep2()) return;

    try {
      setIsLoading(true);
      setError('');

      // Remove confirmPassword before sending to backend
      const { confirmPassword, ...signupData } = formData;

      await authClient.signup(signupData);

      // Refresh session to get user data
      await refreshSession();

      // Call success callback
      if (onSuccess) {
        onSuccess();
      }
    } catch (err: any) {
      setError(err.message || 'Signup failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.formContainer}>
      <div className={styles.formHeader}>
        <h2>Create Account</h2>
        <div className={styles.stepIndicator}>
          <span className={step >= 1 ? styles.stepActive : ''}>1</span>
          <span className={step >= 2 ? styles.stepActive : ''}>2</span>
          <span className={step >= 3 ? styles.stepActive : ''}>3</span>
        </div>
      </div>

      <form onSubmit={handleSubmit} className={styles.form}>
        {/* Step 1: Account Details */}
        {step === 1 && (
          <div className={styles.formStep}>
            <h3>Account Details</h3>

            <div className={styles.formGroup}>
              <label htmlFor="full_name">Full Name</label>
              <input
                type="text"
                id="full_name"
                name="full_name"
                value={formData.full_name}
                onChange={handleChange}
                placeholder="Enter your full name"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="email">Email</label>
              <input
                type="email"
                id="email"
                name="email"
                value={formData.email}
                onChange={handleChange}
                placeholder="Enter your email"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="password">Password</label>
              <input
                type="password"
                id="password"
                name="password"
                value={formData.password}
                onChange={handleChange}
                placeholder="At least 8 characters"
                required
              />
            </div>

            <div className={styles.formGroup}>
              <label htmlFor="confirmPassword">Confirm Password</label>
              <input
                type="password"
                id="confirmPassword"
                name="confirmPassword"
                value={formData.confirmPassword}
                onChange={handleChange}
                placeholder="Re-enter your password"
                required
              />
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <button type="button" onClick={handleNext} className={styles.btnPrimary}>
              Next
            </button>
          </div>
        )}

        {/* Step 2: Software Background */}
        {step === 2 && (
          <div className={styles.formStep}>
            <h3>Software Background</h3>

            <div className={styles.formGroup}>
              <label htmlFor="experience_level">Experience Level</label>
              <select
                id="experience_level"
                name="experience_level"
                value={formData.experience_level}
                onChange={handleChange}
                required
              >
                <option value="">Select your experience level</option>
                {EXPERIENCE_LEVELS.map((level) => (
                  <option key={level} value={level}>
                    {level}
                  </option>
                ))}
              </select>
            </div>

            <div className={styles.formGroup}>
              <label>Programming Languages (select all that apply)</label>
              <div className={styles.checkboxGroup}>
                {PROGRAMMING_LANGUAGES.map((lang) => (
                  <label key={lang} className={styles.checkboxLabel}>
                    <input
                      type="checkbox"
                      checked={formData.programming_languages.includes(lang)}
                      onChange={() => handleCheckboxChange('programming_languages', lang)}
                    />
                    {lang}
                  </label>
                ))}
              </div>
            </div>

            <div className={styles.formGroup}>
              <label>Frameworks & Libraries (optional)</label>
              <div className={styles.checkboxGroup}>
                {FRAMEWORKS.map((framework) => (
                  <label key={framework} className={styles.checkboxLabel}>
                    <input
                      type="checkbox"
                      checked={formData.frameworks.includes(framework)}
                      onChange={() => handleCheckboxChange('frameworks', framework)}
                    />
                    {framework}
                  </label>
                ))}
              </div>
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <div className={styles.buttonGroup}>
              <button type="button" onClick={handleBack} className={styles.btnSecondary}>
                Back
              </button>
              <button type="button" onClick={handleNext} className={styles.btnPrimary}>
                Next
              </button>
            </div>
          </div>
        )}

        {/* Step 3: Hardware Background */}
        {step === 3 && (
          <div className={styles.formStep}>
            <h3>Hardware Background (Optional)</h3>

            <div className={styles.formGroup}>
              <label>Available Compute Hardware</label>
              <div className={styles.checkboxGroup}>
                {HARDWARE_OPTIONS.map((hardware) => (
                  <label key={hardware} className={styles.checkboxLabel}>
                    <input
                      type="checkbox"
                      checked={formData.available_hardware.includes(hardware)}
                      onChange={() => handleCheckboxChange('available_hardware', hardware)}
                    />
                    {hardware}
                  </label>
                ))}
              </div>
            </div>

            <div className={styles.formGroup}>
              <label>Robotics Hardware</label>
              <div className={styles.checkboxGroup}>
                {ROBOTICS_HARDWARE.map((hardware) => (
                  <label key={hardware} className={styles.checkboxLabel}>
                    <input
                      type="checkbox"
                      checked={formData.robotics_hardware.includes(hardware)}
                      onChange={() => handleCheckboxChange('robotics_hardware', hardware)}
                    />
                    {hardware}
                  </label>
                ))}
              </div>
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <div className={styles.buttonGroup}>
              <button type="button" onClick={handleBack} className={styles.btnSecondary}>
                Back
              </button>
              <button type="submit" className={styles.btnPrimary} disabled={isLoading}>
                {isLoading ? 'Creating Account...' : 'Create Account'}
              </button>
            </div>
          </div>
        )}
      </form>
    </div>
  );
};
