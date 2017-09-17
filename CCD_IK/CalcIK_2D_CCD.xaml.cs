using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace CCD_IK
{
	/// <summary>
	/// 2D cyclic coordinate descent based inverse kinematics
	/// </summary>
	public partial class CalcIK_2D_CCD : UserControl, INotifyPropertyChanged
	{
		#region Internal types
		
		// this class represents a bone in it's parent space
		public class BoneData : INotifyPropertyChanged
		{
			private double m_length = 0;
			private double m_angle = 0;

            // joint limiters
            private double m_minLimiters = -3.14;
            private double m_maxLimiters = 3.14;

			#region INotifyPropertyChanged interface
			// event used by the user interface to bind to our properties
			public event PropertyChangedEventHandler PropertyChanged;

			// helper function to notify PropertyChanged subscribers
			protected void NotifyPropertyChanged(string propertyName)
			{
				if (PropertyChanged != null)
				{
					PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
				}
			}
			#endregion
	
			public double Length
			{
				get { return m_length; }
				set { m_length = value; NotifyPropertyChanged("Length"); }
			}
			public double Radians
			{ 
				get { return m_angle; }
				set { m_angle = value; NotifyPropertyChanged("Radians"); NotifyPropertyChanged("Degrees"); }
			}
			public double Degrees
			{ 
				get { return m_angle * 180.0 / Math.PI; }
				set { m_angle = value * Math.PI / 180.0; NotifyPropertyChanged("Radians"); NotifyPropertyChanged("Degrees"); }
			}

            public double MinLimiter
            {
                get { return m_minLimiters; }
                set { m_minLimiters = value * Math.PI / 180.0; NotifyPropertyChanged("MinLimiter"); }
            }
            public double MinLimiterConv
            {
                get { return m_minLimiters * 180.0 / Math.PI; ; }
                set { m_minLimiters = value * Math.PI / 180.0; NotifyPropertyChanged("MinLimiter"); }
            }

            public double MaxLimiter
            {
                get { return m_maxLimiters; }
                set { m_maxLimiters = value * Math.PI / 180.0; NotifyPropertyChanged("Maxlimiter"); }
            }

            public double MaxLimiterConv
            {
                get { return m_maxLimiters * 180.0 / Math.PI; }
                set { m_maxLimiters = value * Math.PI / 180.0; NotifyPropertyChanged("Maxlimiter"); }
            }
		}
		
		#endregion

		#region Private data

		private static Brush[] s_boneLineColors = 
		{
			Brushes.Black,
			Brushes.GreenYellow,
			Brushes.DarkBlue,
			Brushes.DarkRed,
			Brushes.MintCream,
		};

		// number of CCD iterations to perform on each update
		private int m_iterationsPerUpdate = 0;

		// bone lengths
		private ObservableCollection<BoneData> m_bones = new ObservableCollection<BoneData>();

		// lines used to draw the bones
		private List<Line> m_boneLines = new List<Line>();
		
		// target position to reach for
		private Point m_targetPos = new Point(0,0);

		// max distance from end effector to target for a valid solution
		private double m_arrivalDist = 1.0;

		// result of current IK calculation
		private string m_ccdResult = "";

		private DispatcherTimer m_updateTimer;

		#endregion

		#region INotifyPropertyChanged interface
		// event used by the user interface to bind to our properties
		public event PropertyChangedEventHandler PropertyChanged;

		// helper function to notify PropertyChanged subscribers
		protected void NotifyPropertyChanged(string propertyName)
		{
			if (PropertyChanged != null)
			{
				PropertyChanged(this, new PropertyChangedEventArgs(propertyName));
			}
		}
		#endregion

		#region Public properties

		public ObservableCollection<BoneData> Bones
		{
			get { return m_bones; }
		}

		public int IterationsPerUpdate
		{
			get { return m_iterationsPerUpdate; }
			set
			{
				m_iterationsPerUpdate = Math.Max(1,value);
				NotifyPropertyChanged("IterationsPerUpdate"); // update the bound UI
			}
		}

		public string CCDResult
		{
			get { return m_ccdResult; }
			set
			{
				m_ccdResult = value;
				NotifyPropertyChanged("CCDResult"); // update the bound UI
			}
		}
			
		public double TargetPosX
		{
			get { return m_targetPos.X; }
			set
			{
				m_targetPos.X = value;
				NotifyPropertyChanged("TargetPosX"); // update the bound UI
				UpdateDisplay(); // redraw
			}
		}

		public double TargetPosY
		{
			get { return m_targetPos.Y; }
			set
			{
				m_targetPos.Y = value;
				NotifyPropertyChanged("TargetPosY"); // update the bound UI
				UpdateDisplay(); // redraw
			}
		}

		public double ArrivalDist
		{
			get { return m_arrivalDist; }
			set
			{
				m_arrivalDist = value;
				NotifyPropertyChanged("ArrivalDist"); // update the bound UI
				UpdateDisplay(); // redraw
			}
		}

		#endregion

		#region Lifespan functions
		public CalcIK_2D_CCD()
		{
			InitializeComponent();
			
			// set the iteration number
			IterationsPerUpdate = 1;

			// create the timer
			m_updateTimer = new DispatcherTimer(DispatcherPriority.Normal);
			m_updateTimer.Tick += new EventHandler(UpdateTimer_Tick);
			m_updateTimer.Interval = new TimeSpan(0, 0, 0, 0, 100);

			// add the initial bones
			AddBone();
			AddBone();
            AddBone();
            AddBone();


			TargetPosX = 100;
			TargetPosY = 100;

			// update the display
			UpdateDisplay();
		}
		#endregion
		
		#region Coordinate Conversion

		// compute the logical origin in _viewport coordinated
		private double ViewportWidth { get { return _viewportColumn.ActualWidth; } }
		private double ViewportHieght { get { return _mainGrid.ActualHeight; } }
		private double ViewportCenterX { get { return ViewportWidth / 2; } }
		private double ViewportCenterY { get { return ViewportHieght / 2; } }

		// convert logical coordinates to _viewport coordinates
		private double LogicalToViewportX(double logicalX) { return logicalX + ViewportCenterX; }
		private double LogicalToViewportY(double logicalY) { return -logicalY + ViewportCenterY; }
	
		// convert _viewport coordinates to logical coordinates
		private double ViewportToLogicalX(double viewportX) { return viewportX - ViewportCenterX; }
		private double ViewportToLogicalY(double viewportY) { return -viewportY + ViewportCenterY; }
		
		#endregion

		#region Logic Functions

		/// <summary>
		/// Add a new bone to the chain at the selected location or at the end if no location is selected.
		/// </summary>
		void AddBone()
		{
			BoneData newBone = new BoneData();
			newBone.Length = 50;

			newBone.PropertyChanged += BonePropertyChanged;

			// insert at the end if no bone is selected
			if( _boneList.SelectedIndex == -1 )
				Bones.Add( newBone );
			else
				Bones.Insert( _boneList.SelectedIndex, newBone );
		}

		/// <summary>
		/// Remove a new bone from the chain at the selected location or from the end if no location is selected.
		/// </summary>
		private void RemoveBone()
		{
			if( Bones.Count == 0 )
				return;

			// remove the end bone if no bone is selected
			int removeIdx = _boneList.SelectedIndex;
			if( removeIdx == -1 )
				removeIdx = (Bones.Count - 1);


			Bones[removeIdx].PropertyChanged -= BonePropertyChanged;
			Bones.RemoveAt( removeIdx );
		}

		/// <summary>
		/// Perform an iteration of IK
		/// </summary>
		private void UpdateIK()
		{
			int numBones = Bones.Count;
			
			if( numBones == 0 )
				return;

			// calculate the bone angles
			List< IKSolver.Bone_2D_CCD > ccdBones = new List< IKSolver.Bone_2D_CCD >();
			for( int boneIdx = 0; boneIdx <= numBones; ++boneIdx )
			{
				IKSolver.Bone_2D_CCD newCcdBone = new IKSolver.Bone_2D_CCD();
				newCcdBone.angle	= (boneIdx < numBones) ? Bones[boneIdx].Radians : 0;
				newCcdBone.x		= (boneIdx > 0) ? Bones[boneIdx-1].Length : 0;
				newCcdBone.y		= 0;
                newCcdBone.minlimiter = (boneIdx < numBones) ? Bones[boneIdx].MinLimiter : 0;
                newCcdBone.maxlimiter = (boneIdx < numBones) ? Bones[boneIdx].MaxLimiter : 0;
				ccdBones.Add( newCcdBone );
			}

			// iterate CCD until limit is reached or we find a valid solution
			for( int itrCount = 0; itrCount < IterationsPerUpdate; ++itrCount )
			{
				IKSolver.CCDResult result = IKSolver.CalcIK_2D_CCD( ref ccdBones, TargetPosX, TargetPosY, ArrivalDist );
				if( result == IKSolver.CCDResult.Processing )
				{
					CCDResult = "Обработка";
				}
				else if( result == IKSolver.CCDResult.Success )
				{
					CCDResult = "Выполнено";
					break;
				}
				else if( result == IKSolver.CCDResult.Failure )
				{
					CCDResult = "Неудача";
					break;
				}
				else
				{
					Debug.Assert(false);
					CCDResult = "[UNKNOWN]";
					break;
				}
			}

			// extract the new bone data from the results
			for( int boneIdx = 0; boneIdx < numBones; ++boneIdx )
			{
				Bones[boneIdx].Radians = ccdBones[boneIdx].angle;
			}
		}

		/// <summary>
		/// Update the scene displayed in the viewport
		/// </summary>
		private void UpdateDisplay()
		{
			int numBones = Bones.Count;
			
			// resize the number of bone lines
			while( m_boneLines.Count > numBones )
			{
				_viewport.Children.Remove( m_boneLines[m_boneLines.Count-1] );
				m_boneLines.RemoveAt( m_boneLines.Count-1 );
			}

			while( m_boneLines.Count < numBones )
			{
				Line newBoneLine = new Line();
				newBoneLine.Stroke = s_boneLineColors[m_boneLines.Count % s_boneLineColors.Length];
				newBoneLine.StrokeThickness	= 3;
				newBoneLine.SetValue( Panel.ZIndexProperty, 100 );

				m_boneLines.Add( newBoneLine );
				_viewport.Children.Add( newBoneLine );
			}

			// compute the orientations of the bone lines in logical space
			double curAngle = 0;
			for( int boneIdx = 0; boneIdx < numBones; ++boneIdx )
			{
				BoneData curBone = Bones[boneIdx];

				curAngle += curBone.Radians;
				double cosAngle = Math.Cos( curAngle );
				double sinAngle = Math.Sin( curAngle );

				if( boneIdx > 0 )
				{
					m_boneLines[boneIdx].X1 = m_boneLines[boneIdx-1].X2;
					m_boneLines[boneIdx].Y1 = m_boneLines[boneIdx-1].Y2;
				}
				else
				{
					m_boneLines[boneIdx].X1 = 0;
					m_boneLines[boneIdx].Y1 = 0;
				}

				m_boneLines[boneIdx].X2 = m_boneLines[boneIdx].X1 + cosAngle*curBone.Length;
				m_boneLines[boneIdx].Y2 = m_boneLines[boneIdx].Y1 + sinAngle*curBone.Length;
			}

			// convert the bone positions to viewport space
			foreach( Line curLine in m_boneLines )
			{
				curLine.X1 = LogicalToViewportX(curLine.X1);
				curLine.Y1 = LogicalToViewportY(curLine.Y1);

				curLine.X2 = LogicalToViewportX(curLine.X2);
				curLine.Y2 = LogicalToViewportY(curLine.Y2);
			}

			// draw the arrival distance
			Canvas.SetLeft( _arrivalEllipse, LogicalToViewportX(TargetPosX - ArrivalDist) );
			Canvas.SetTop( _arrivalEllipse, LogicalToViewportY(TargetPosY + ArrivalDist) );
			_arrivalEllipse.Width = 2.0 * ArrivalDist;
			_arrivalEllipse.Height = 2.0 * ArrivalDist;

			// draw the target
			Canvas.SetLeft( _targetEllipse, LogicalToViewportX(TargetPosX - _targetEllipse.Width/2) );
			Canvas.SetTop( _targetEllipse, LogicalToViewportY(TargetPosY + _targetEllipse.Height/2) );
			
			// draw the axes
			_xAxisLine.X1 = 0;
			_xAxisLine.Y1 = ViewportCenterY;
			_xAxisLine.X2 = ViewportWidth;
			_xAxisLine.Y2 = ViewportCenterY;

			_yAxisLine.X1 = ViewportCenterX;
			_yAxisLine.Y1 = 0;
			_yAxisLine.X2 = ViewportCenterX;
			_yAxisLine.Y2 = ViewportHieght;
		}

		/// <summary>
		/// Update logic at a set interval
		/// </summary>
		private void UpdateTimer_Tick(object sender, EventArgs e)
        {
			UpdateIK();
			UpdateDisplay();
		}

		#endregion
		
		#region Event Handlers

		private void BonePropertyChanged(object sender, PropertyChangedEventArgs e)
		{
			switch (e.PropertyName)
			{
				case "Radians":
					UpdateDisplay();
					break;
				case "Length":
					UpdateDisplay();
					break;
			}
		}

		private void viewport_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
		{
			// capture the mouse to keep grabing MouseMove events if the user drags the
			// mouse outside of the _viewport bounds
			if (!_viewport.IsMouseCaptured)
            {
                _viewport.CaptureMouse();
			}

			// update the target position
			Point viewportPos = e.GetPosition(_viewport);
			TargetPosX = ViewportToLogicalX( viewportPos.X );
			TargetPosY = ViewportToLogicalY( viewportPos.Y );
		}

		private void viewport_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
		{
			// release the captured mouse
			if (_viewport.IsMouseCaptured)
            {
                _viewport.ReleaseMouseCapture();
            }
		}

		private void viewport_MouseMove(object sender, MouseEventArgs e)
		{
			// update the target position if we are still in a captured state
			// (i.e. the user has not released the mouse button)
			if (_viewport.IsMouseCaptured)
            {
				Point viewportPos = e.GetPosition(_viewport);
				TargetPosX = ViewportToLogicalX( viewportPos.X );
				TargetPosY = ViewportToLogicalY( viewportPos.Y );
            }
		}

		private void _thisWindow_SizeChanged(object sender, SizeChangedEventArgs e)
		{
			// update the display shapes based on the new window size
			UpdateDisplay();
		}
		
		private void _thisWindow_IsVisibleChanged(object sender, DependencyPropertyChangedEventArgs e)
		{			
			if( m_updateTimer != null )
			{
				if( this.IsVisible )
				{
					if( _playRadioButton.IsChecked == true )
						m_updateTimer.Start();
				}
				else
				{
					m_updateTimer.Stop();
				}
			}
		}

		private void _websiteLink_Click(object sender, RoutedEventArgs e)
		{
			System.Diagnostics.Process.Start( "http://www.ryanjuckett.com" );
		}

		private void _addBoneButton_Click(object sender, RoutedEventArgs e)
		{
			AddBone();
			UpdateDisplay();
		}

		private void _removeBoneButton_Click(object sender, RoutedEventArgs e)
		{
			RemoveBone();
			UpdateDisplay();
		}	

		private void _playRadioButton_Checked(object sender, RoutedEventArgs e)
		{
			if( m_updateTimer != null )
				m_updateTimer.Start();
		}

		private void _pauseRadioButton_Checked(object sender, RoutedEventArgs e)
		{
			if( m_updateTimer != null )
				m_updateTimer.Stop();
		}

		private void _singleUpdateButton_Click(object sender, RoutedEventArgs e)
		{
			_pauseRadioButton.IsChecked = true;
			UpdateIK();
			UpdateDisplay();
		}

		#endregion
	}
}
